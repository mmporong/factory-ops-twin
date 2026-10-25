#requires -Version 7.2
[CmdletBinding()]
param(
    [string]$CredentialFile = (Join-Path $HOME '.config/factoryops/ngc-api-key.dpapi'),
    [Parameter(Mandatory = $true)][string]$OutputDirectory
)

$ErrorActionPreference = 'Stop'
$ProgressPreference = 'SilentlyContinue'
$outputRoot = [IO.Path]::GetFullPath($OutputDirectory)
if (Test-Path -LiteralPath $outputRoot) { throw 'Use a new output directory.' }
[IO.Directory]::CreateDirectory($outputRoot) | Out-Null
$record = [ordered]@{
    schema_version = 1
    checked_utc = [DateTime]::UtcNow.ToString('o')
    image = 'nvcr.io/nim/nvidia/cosmos3:2.1.0'
    scope = 'Credential authentication and registry manifest metadata only; no image layers, models or GPU execution.'
    credential_decrypted = $false
    authentication_http_status = $null
    manifest_http_status = $null
    image_digest = $null
    credentials_or_tokens_logged = $false
    status = 'starting'
}
$secureKey = $null
$plainKey = $null
$basicValue = $null
$bearerToken = $null
$tokenData = $null
$request = $null
$response = $null
$handler = [Net.Http.HttpClientHandler]::new()
$handler.AllowAutoRedirect = $false
$client = [Net.Http.HttpClient]::new($handler)
$client.Timeout = [TimeSpan]::FromSeconds(45)
try {
    $encryptedKey = (Get-Content -LiteralPath $CredentialFile -Raw).Trim()
    $secureKey = ConvertTo-SecureString -String $encryptedKey
    $plainKey = [Net.NetworkCredential]::new('', $secureKey).Password.Trim()
    if ([string]::IsNullOrWhiteSpace($plainKey)) { throw 'The stored credential is empty.' }
    $record.credential_decrypted = $true
    $basicValue = [Convert]::ToBase64String([Text.Encoding]::UTF8.GetBytes('$oauthtoken:' + $plainKey))
    $request = [Net.Http.HttpRequestMessage]::new('GET', 'https://nvcr.io/proxy_auth?service=nvcr.io&scope=repository%3Anim%2Fnvidia%2Fcosmos3%3Apull')
    $request.Headers.Authorization = [Net.Http.Headers.AuthenticationHeaderValue]::new('Basic', $basicValue)
    $response = $client.SendAsync($request).GetAwaiter().GetResult()
    $record.authentication_http_status = [int]$response.StatusCode
    if (-not $response.IsSuccessStatusCode) { throw 'NGC authentication did not succeed.' }
    $tokenData = $response.Content.ReadAsStringAsync().GetAwaiter().GetResult() | ConvertFrom-Json
    $bearerToken = if ($tokenData.PSObject.Properties.Name -contains 'token') { $tokenData.token } else { $tokenData.access_token }
    if ([string]::IsNullOrWhiteSpace($bearerToken)) { throw 'NGC returned no bearer token.' }
    $request.Dispose()
    $response.Dispose()
    $request = [Net.Http.HttpRequestMessage]::new('GET', 'https://nvcr.io/v2/nim/nvidia/cosmos3/manifests/2.1.0')
    $request.Headers.Authorization = [Net.Http.Headers.AuthenticationHeaderValue]::new('Bearer', $bearerToken)
    $request.Headers.TryAddWithoutValidation('Accept', 'application/vnd.oci.image.index.v1+json, application/vnd.docker.distribution.manifest.list.v2+json, application/vnd.oci.image.manifest.v1+json, application/vnd.docker.distribution.manifest.v2+json') | Out-Null
    $response = $client.SendAsync($request).GetAwaiter().GetResult()
    $record.manifest_http_status = [int]$response.StatusCode
    if (-not $response.IsSuccessStatusCode) { throw 'The requested image manifest is not accessible.' }
    $bodyBytes = $response.Content.ReadAsByteArrayAsync().GetAwaiter().GetResult()
    $manifest = [Text.Encoding]::UTF8.GetString($bodyBytes) | ConvertFrom-Json
    $record.image_digest = 'sha256:' + [Convert]::ToHexString([Security.Cryptography.SHA256]::HashData($bodyBytes)).ToLowerInvariant()
    $record.manifest_bytes = $bodyBytes.Length
    $record.media_type = $manifest.mediaType
    $record.status = 'authenticated_manifest_accessible'
}
catch {
    $record.status = 'failed'
    # Exception text and HTTP bodies may contain authentication context.
    $record.error_type = $_.Exception.GetType().FullName
}
finally {
    if ($secureKey) { $secureKey.Dispose() }
    if ($request) { $request.Dispose() }
    if ($response) { $response.Dispose() }
    $client.Dispose()
    $handler.Dispose()
    $plainKey = $null
    $encryptedKey = $null
    $basicValue = $null
    $bearerToken = $null
    $tokenData = $null
    $json = $record | ConvertTo-Json -Depth 6
    [IO.File]::WriteAllText((Join-Path $outputRoot 'result.json'), $json)
    $json
}
if ($record.status -ne 'authenticated_manifest_accessible') { exit 1 }
