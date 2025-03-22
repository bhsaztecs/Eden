Write-Host "WARNING: This code is AI generated. I don't own a windows machine, i have no idea if this will work."
function Initialize {
    Write-Host "save before initialization? (y/n)"
    $save = Read-Host
    if ($save -eq "Y" -or $save -eq "y") {
        git init
        git add .
        git commit -m "pre-initialize"
    }
    elseif ($save -ne "N" -and $save -ne "n") {
        Write-Host "Invalid input"
    }
    
    New-Item -Path "bin" -ItemType Directory -Force
    New-Item -Path "bin/botball_user_program" -ItemType File -Force
    docker pull sillyfreak/wombat-cross
    New-Item -Path "src/main.cpp" -ItemType File -Force
    Get-Content -Path "data/example.cpp" | Set-Content -Path "src/main.cpp"
    Write-Host "Run 'compile --nocopy executable' to validate."
}

function Executable {
    New-Item -Path "develop" -ItemType Directory -Force
    New-Item -Path "bin" -ItemType Directory -Force
    Copy-Item -Path "bin", "src", "include", "data", "LICENSE.txt", "project.manifest", "README.md" -Destination "develop/" -Recurse -Force
    docker run -it --rm --volume ${PWD}/develop:/home/kipr:rw sillyfreak/wombat-cross `
        aarch64-linux-gnu-g++ -std=c++17 -g -Wall -I./include/* -I/usr/local/include/include/ -L/usr/local/lib ./src/*.cpp -lm -lpthread -lkipr -lz -o ./bin/botball_user_program
    Copy-Item -Path "./develop/bin/botball_user_program" -Destination "bin/" -Force
    Remove-Item -Path "develop" -Recurse -Force
}

function Library {
    New-Item -Path "develop" -ItemType Directory -Force
    New-Item -Path "bin" -ItemType Directory -Force
    Copy-Item -Path "bin", "src", "include", "data", "LICENSE.txt", "project.manifest", "README.md" -Destination "develop/" -Recurse -Force
    docker run -it --rm --volume ${PWD}/develop:/home/kipr:rw sillyfreak/wombat-cross `
        aarch64-linux-gnu-g++ -std=c++17 -g -fPIC -shared -Wall -I./include/* -I/usr/local/include/include/ -L/usr/local/lib ./src/*.cpp -lm -lpthread -lkipr -lz -o ./bin/libeden.so
    Copy-Item -Path "./develop/bin/libeden.so" -Destination "bin/" -Force
    Remove-Item -Path "develop" -Recurse -Force
}

function Copy-Files {
    param (
        [Parameter(Mandatory=$true)]
        [string]$destination
    )
    
    $pingResult = Test-Connection -ComputerName 192.168.125.1 -Count 1 -Quiet
    if (-not $pingResult) {
        Write-Host "Host unreachable"
        return $false
    }
    
    try {
        scp -r * "kipr@192.168.125.1:~/Documents/KISS/$destination"
        return $true
    }
    catch {
        return $false
    }
}

function Remote-Shell {
    ssh kipr@192.168.125.1
}

function Compile {
    param (
        [Parameter(Mandatory=$true)]
        [string]$destination,
        [Parameter(Mandatory=$false)]
        [string]$buildType
    )
    
    if ([string]::IsNullOrEmpty($destination)) {
        Write-Host "No destination specified."
        return $false
    }
    
    Write-Host "Compiling..."
    
    $compileSuccess = $false
    if ($buildType -eq "executable") {
        $compileSuccess = Executable
    }
    elseif ($buildType -eq "library") {
        $compileSuccess = Library
    }
    
    if ($compileSuccess) {
        if ($destination -ne "--nocopy") {
            if (Copy-Files -destination $destination) {
                Write-Host -NoNewline "Succeeded at: "
            }
            else {
                Write-Host -NoNewline "Copy failed at: "
            }
        }
        else {
            Write-Host -NoNewline "Compilation succeeded at: "
        }
    }
    else {
        if (Test-Path -Path "develop") {
            Remove-Item -Path "develop" -Recurse -Force
        }
        Write-Host -NoNewline "Compilation failed at: "
    }
    
    Get-Date -Format "hh:mm:ss tt"
}
