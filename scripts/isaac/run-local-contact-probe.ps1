param(
    [Parameter(Mandatory)][string]$SdkPath,
    [Parameter(Mandatory)][string]$Config,
    [Parameter(Mandatory)][string]$OutputDirectory,
    [Parameter(Mandatory)][ValidatePattern('^[0-9a-f]{32}$')][string]$RunId,
    [string]$EntryPoint = 'local_contact_probe.py',
    [string[]]$SourceDependency = @(),
    [ValidateRange(60,480)][int]$TimeoutSec = 480
)
$ErrorActionPreference = 'Stop'
$sdkRoot = (Resolve-Path -LiteralPath $SdkPath).Path
$configPath = (Resolve-Path -LiteralPath $Config).Path
$probeScript = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot $EntryPoint)).Path
if ([IO.Path]::GetDirectoryName($probeScript) -ne $PSScriptRoot -or [IO.Path]::GetExtension($probeScript) -ne '.py') {
    throw 'Entry point must be an existing sibling Python probe'
}
$probeName = [IO.Path]::GetFileName($probeScript)
$outputPath = [IO.Path]::GetFullPath($OutputDirectory)
if ((Test-Path -LiteralPath $outputPath) -or (Test-Path -LiteralPath ($outputPath + '.host.json'))) {
    throw 'Use a new output directory and host receipt'
}
$outputParent = Split-Path -Parent $outputPath
New-Item -ItemType Directory -Force -Path $outputParent | Out-Null
$launcher = Join-Path $sdkRoot 'python.bat'
if (-not (Test-Path -LiteralPath $launcher)) { throw 'Existing vendor Python launcher missing' }
$snapshotPath = $outputPath + '.inputs'
if (Test-Path -LiteralPath $snapshotPath) { throw 'Input snapshot already exists; use a new attempt' }
New-Item -ItemType Directory -Path $snapshotPath | Out-Null
Copy-Item -LiteralPath $probeScript -Destination (Join-Path $snapshotPath $probeName)
Copy-Item -LiteralPath $configPath -Destination (Join-Path $snapshotPath 'config.json')
Copy-Item -LiteralPath $PSCommandPath -Destination (Join-Path $snapshotPath 'launcher.ps1')
$projectRoot = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot '../..'))
$dependencyHashes = [ordered]@{}
foreach ($relative in $SourceDependency) {
    $dependencyPath = (Resolve-Path -LiteralPath (Join-Path $projectRoot $relative)).Path
    if (-not $dependencyPath.StartsWith($projectRoot + [IO.Path]::DirectorySeparatorChar, [StringComparison]::OrdinalIgnoreCase) -or [IO.Path]::GetExtension($dependencyPath) -ne '.py') {
        throw 'Source dependencies must be Python files within the project'
    }
    $normalized = [IO.Path]::GetRelativePath($projectRoot, $dependencyPath).Replace('\','/')
    $copyPath = Join-Path (Join-Path $snapshotPath 'project') $normalized
    New-Item -ItemType Directory -Force -Path (Split-Path -Parent $copyPath) | Out-Null
    Copy-Item -LiteralPath $dependencyPath -Destination $copyPath
    $dependencyHashes[$normalized] = (Get-FileHash -LiteralPath $copyPath -Algorithm SHA256).Hash.ToLower()
}
$arguments = '""{0}" "{1}" --config "{2}" --output "{3}" --run-id {4}"' -f $launcher,$probeScript,$configPath,$outputPath,$RunId
$started = [DateTime]::UtcNow
$watch = [Diagnostics.Stopwatch]::StartNew()
$worker = $null
$receipt = [ordered]@{ run_id=$RunId; started_at_utc=$started.ToString('o'); scope='native bounded component probe'; timeout_s=$TimeoutSec; sdk_root=$sdkRoot; entry_point=$probeName; inputs_directory=$snapshotPath; launcher_sha256=(Get-FileHash -LiteralPath (Join-Path $snapshotPath 'launcher.ps1') -Algorithm SHA256).Hash.ToLower(); source_sha256=(Get-FileHash -LiteralPath (Join-Path $snapshotPath $probeName) -Algorithm SHA256).Hash.ToLower(); gpu=( & nvidia-smi --query-gpu=name,memory.total,driver_version --format=csv,noheader ); timed_out=$false; remaining_owned_pids=@() }
$receipt['config_sha256'] = (Get-FileHash -LiteralPath (Join-Path $snapshotPath 'config.json') -Algorithm SHA256).Hash.ToLower()
$receipt['project_source_sha256'] = $dependencyHashes
$receipt['closure_scope'] = 'launcher and run-matched immediate children; full descendant tree is not guaranteed'
$receipt['full_descendant_closure_verified'] = $false
$receipt['timeout_full_tree_cleanup_supported'] = $false
try {
    $worker = Start-Process -FilePath $env:ComSpec -ArgumentList '/d','/s','/c',$arguments -WorkingDirectory $sdkRoot -WindowStyle Hidden -RedirectStandardOutput ($outputPath + '.log') -RedirectStandardError ($outputPath + '.error.log') -PassThru
    $receipt['launcher_pid'] = $worker.Id
    $receipt | ConvertTo-Json -Depth 8 | Set-Content -LiteralPath ($outputPath + '.host.json') -Encoding utf8
    while (-not $worker.HasExited -and $watch.Elapsed.TotalSeconds -lt $TimeoutSec) {
        Start-Sleep -Milliseconds 500
        $worker.Refresh()
    }
    $receipt['timed_out'] = -not $worker.HasExited
    if ($worker.HasExited) { $worker.WaitForExit(); $receipt['exit_code'] = $worker.ExitCode }
} catch {
    $receipt['host_error'] = $_.Exception.Message
} finally {
    $owned = @(Get-CimInstance Win32_Process | Where-Object { $worker -and $_.ProcessId -ne $PID -and ($_.ProcessId -eq $worker.Id -or $_.ParentProcessId -eq $worker.Id) -and $_.CreationDate -and $_.CommandLine -and $_.CreationDate.ToUniversalTime() -ge $started.AddSeconds(-1) -and $_.CommandLine -like "*$RunId*" -and $_.CommandLine -like "*$probeName*" })
    $receipt['owned_pids_at_cleanup'] = @($owned | ForEach-Object ProcessId)
    foreach ($process in $owned) { Stop-Process -Id $process.ProcessId -ErrorAction SilentlyContinue }
    if ($worker -and -not $worker.HasExited) { $worker.WaitForExit(5000) | Out-Null }
    $receipt['remaining_owned_pids'] = @(Get-CimInstance Win32_Process | Where-Object { $worker -and $_.ProcessId -ne $PID -and ($_.ProcessId -eq $worker.Id -or $_.ParentProcessId -eq $worker.Id) -and $_.CreationDate -and $_.CommandLine -and $_.CreationDate.ToUniversalTime() -ge $started.AddSeconds(-1) -and $_.CommandLine -like "*$RunId*" -and $_.CommandLine -like "*$probeName*" } | ForEach-Object ProcessId)
    $receipt['closed'] = $receipt.remaining_owned_pids.Count -eq 0
    try {
        $probeResult = Get-Content -LiteralPath (Join-Path $outputPath 'result.json') -Raw | ConvertFrom-Json
        $receipt['probe_status'] = $probeResult.status
        $receipt['probe_identity_matched'] = $probeResult.run_id -eq $RunId -and $probeResult.source_sha256 -eq $receipt.source_sha256 -and $probeResult.config_sha256 -eq $receipt.config_sha256
        $reportedKeys = @($probeResult.project_source_sha256.PSObject.Properties | ForEach-Object Name)
        $dependenciesMatched = $reportedKeys.Count -eq $dependencyHashes.Count
        foreach ($relative in $dependencyHashes.Keys) {
            if ($probeResult.project_source_sha256.$relative -ne $dependencyHashes[$relative]) { $dependenciesMatched = $false }
        }
        $receipt['project_sources_matched'] = $dependenciesMatched
    } catch {
        $receipt['probe_status'] = 'invalid_or_missing'
        $receipt['result_read_error'] = $_.Exception.Message
        $receipt['probe_identity_matched'] = $false
    }
    $receipt['finished_at_utc'] = [DateTime]::UtcNow.ToString('o')
    $receipt['wall_time_s'] = $watch.Elapsed.TotalSeconds
    $receipt | ConvertTo-Json -Depth 8 | Set-Content -LiteralPath ($outputPath + '.host.json') -Encoding utf8
}
if ($receipt.timed_out -or $receipt.host_error -or $receipt.exit_code -ne 0 -or -not $receipt.closed -or $receipt.probe_status -ne 'passed' -or -not $receipt.probe_identity_matched -or -not $receipt.project_sources_matched) { exit 1 }
