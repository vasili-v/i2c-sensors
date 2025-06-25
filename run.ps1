Param (
    [Parameter(Mandatory)]
    [string]$Command,

    [string]$Destination
)

$VirtualEnvPath = ".venv"
$Python = "$VirtualEnvPath\Scripts\python.exe"

$SCP = "C:\Program Files\Git\usr\bin\scp.exe"

switch ($Command.ToLower()) {
    "printenv" {
        if (-Not (Test-Path "$VirtualEnvPath")) {
            $Python = "python"
        }

        & "$Python" -c "import sys; import os; print(f'Interpreter: {sys.executable}\nPYTHONPATH.: {os.environ.get('PYTHONPATH', '')}')"
    }
    "lint" {
        Write-Host "Running linter..."
        if (-Not (Test-Path "$VirtualEnvPath")) {
            Write-Error """$VirtualEnvPath"" doesn't exist. Exiting..."
            return
        }

        & "$Python" -m pylint ".\src"
    }
    "build" {
        Write-Host "Running build..."
        if (-Not (Test-Path "$VirtualEnvPath")) {
            Write-Error """$VirtualEnvPath"" doesn't exist. Exiting..."
            return
        }

        & "$Python" -m build
    }
    "push" {
        Write-Host "Copying package to $Destination..."
        & "$SCP" ./dist/i2cs-0.0.1-py3-none-any.whl "$Destination"
    }
    default {
        Write-Error "Unknown command: ""$Command"""
        return 1
    }
}
