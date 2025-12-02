@echo off
REM Database Setup Script for Physical AI Auth Backend (Windows)
REM This script helps set up PostgreSQL database and run migrations

setlocal enabledelayedexpansion

echo.
echo 🗄️  Physical AI Auth Backend - Database Setup
echo =============================================
echo.

REM Check if .env file exists
if not exist ".env" (
    echo ❌ Error: .env file not found
    echo Please create .env from .env.example first
    exit /b 1
)

REM Parse .env file for DATABASE_URL
for /f "tokens=1,2 delims==" %%A in (.env) do (
    if "%%A"=="DATABASE_URL" set DATABASE_URL=%%B
)

REM Check if DATABASE_URL is set
if "%DATABASE_URL%"=="" (
    echo ❌ Error: DATABASE_URL not set in .env
    exit /b 1
)

echo ✅ Database URL configured
echo.

REM Check if psql is installed
where psql >nul 2>nul
if %ERRORLEVEL% NEQ 0 (
    echo ⚠️  PostgreSQL client (psql) not found
    echo Install PostgreSQL from: https://www.postgresql.org/download/windows/
    echo Make sure to add PostgreSQL bin folder to PATH
    exit /b 1
)

echo Testing database connection...
psql -c "SELECT 1" "%DATABASE_URL%" >nul 2>&1
if %ERRORLEVEL% EQU 0 (
    echo ✅ Database connection successful
) else (
    echo ❌ Cannot connect to database
    echo Check your DATABASE_URL in .env
    exit /b 1
)

echo.
echo Running migrations...
echo ---------------------
echo.

REM Run migration
psql -f migrations/0001_initial_schema.sql "%DATABASE_URL%" 2>nul
if %ERRORLEVEL% EQU 0 (
    echo ✅ Migrations completed successfully
) else (
    echo ⚠️  Migration completed with warnings (tables may already exist)
)

echo.
echo Verifying database schema...
echo ----------------------------
echo.

REM Check tables
echo Tables created:
psql -c "\dt" "%DATABASE_URL%"

echo.
echo ✅ Database setup complete!
echo.
echo Next steps:
echo 1. Start the server: npm run dev
echo 2. Test health endpoint: curl http://localhost:3001/health
echo 3. Sign up a user and test the API
echo.

pause
