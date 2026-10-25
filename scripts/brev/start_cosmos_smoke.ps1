#requires -Version 7.2
[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)][string]$OutputDirectory,
    [Parameter(Mandatory = $true)][string]$InputsDirectory,
    [string]$CredentialFile = (Join-Path $HOME '.config/factoryops/ngc-api-key.dpapi'),
    [switch]$Execute
)

$ErrorActionPreference = 'Stop'
$repoRoot = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot '../..'))
$pythonPath = Join-Path $repoRoot '.venv/Scripts/python.exe'
$controllerPath = Join-Path $PSScriptRoot 'run_cosmos_smoke.py'
$remoteScript = Join-Path $PSScriptRoot 'run-cosmos-smoke.sh'
foreach ($requiredFile in @($pythonPath, $controllerPath, $remoteScript)) {
    if (-not (Test-Path -LiteralPath $requiredFile -PathType Leaf)) { throw 'A required launcher file is missing.' }
}
$workerInfo = [Diagnostics.ProcessStartInfo]::new()
$workerInfo.FileName = $pythonPath
$workerInfo.UseShellExecute = $false
$workerInfo.WorkingDirectory = $repoRoot
$workerInfo.RedirectStandardInput = $true
foreach ($workerArgument in @('-X', 'utf8', $controllerPath,
    '--output', [IO.Path]::GetFullPath($OutputDirectory),
    '--remote-script', $remoteScript,
    '--inputs-directory', [IO.Path]::GetFullPath($InputsDirectory))) {
    $workerInfo.ArgumentList.Add($workerArgument)
}
$secureKey = $null
$plainKey = $null
$worker = $null
try {
    if ($Execute) {
        $encryptedKey = (Get-Content -LiteralPath $CredentialFile -Raw).Trim()
        $secureKey = ConvertTo-SecureString -String $encryptedKey
        $plainKey = [Net.NetworkCredential]::new('', $secureKey).Password.Trim()
        if ([string]::IsNullOrWhiteSpace($plainKey)) { throw 'The stored credential is empty.' }
        $workerInfo.ArgumentList.Add('--execute')
        $workerInfo.ArgumentList.Add('--key-stdin')
    }
    $worker = [Diagnostics.Process]::Start($workerInfo)
    if ($Execute) { $worker.StandardInput.WriteLine($plainKey) }
    $worker.StandardInput.Close()
    $plainKey = $null
    if ($secureKey) { $secureKey.Dispose(); $secureKey = $null }
    $encryptedKey = $null
    $worker.WaitForExit()
    $workerExitCode = $worker.ExitCode
}
finally {
    if ($secureKey) { $secureKey.Dispose() }
    $plainKey = $null
    $encryptedKey = $null
    if ($worker) { $worker.Dispose() }
}
exit $workerExitCode
