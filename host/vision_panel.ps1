param([switch]$SelfTest,[switch]$SmokeTest)
& (Join-Path $PSScriptRoot 'vision_panel_cn.ps1') @PSBoundParameters