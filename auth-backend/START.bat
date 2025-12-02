@echo off
REM Better Auth - Quick Start Script for Windows

echo.
echo ╔════════════════════════════════════════════════════════════╗
echo ║         Better Auth - Local Development Starter            ║
echo ║          Physical AI Textbook Authentication               ║
echo ╚════════════════════════════════════════════════════════════╝
echo.

REM Check if PostgreSQL container exists
docker ps -a | findstr "physical-ai-auth-db" >nul
if %ERRORLEVEL% NEQ 0 (
    echo ⚠️  PostgreSQL container not found.
    echo.
    echo Starting PostgreSQL in Docker...
    echo.
    docker run --name physical-ai-auth-db ^
      -e POSTGRES_USER=authuser ^
      -e POSTGRES_PASSWORD=authpass123 ^
      -e POSTGRES_DB=physical_ai_auth ^
      -p 5432:5432 ^
      -d postgres:16-alpine
    echo.
    echo ✅ PostgreSQL started!
    echo.
    echo Waiting 5 seconds for database to initialize...
    timeout /t 5 /nobreak
) else (
    echo.
    echo Checking PostgreSQL status...
    docker ps | findstr "physical-ai-auth-db" >nul
    if %ERRORLEVEL% EQ 0 (
        echo ✅ PostgreSQL is already running
    ) else (
        echo Starting PostgreSQL container...
        docker start physical-ai-auth-db
        timeout /t 3 /nobreak
    )
)

echo.
echo 📦 Installing dependencies...
call npm install
echo.

echo.
echo 🚀 Starting Better Auth Backend...
echo.
echo    Backend URL: http://localhost:3001
echo    Test UI: Open test-client.html in your browser
echo.
echo 📖 For detailed guide, see: LOCAL_SETUP_GUIDE.md
echo.

npm run dev

pause
