#!/bin/bash

# Database Setup Script for Physical AI Auth Backend
# This script helps set up PostgreSQL database and run migrations

set -e

echo "🗄️  Physical AI Auth Backend - Database Setup"
echo "=============================================="
echo ""

# Check if .env file exists
if [ ! -f .env ]; then
    echo "❌ Error: .env file not found"
    echo "Please create .env from .env.example first"
    exit 1
fi

# Load environment variables
export $(grep -v '^#' .env | xargs)

# Check if DATABASE_URL is set
if [ -z "$DATABASE_URL" ]; then
    echo "❌ Error: DATABASE_URL not set in .env"
    exit 1
fi

echo "✅ Database URL configured"
echo ""

# Check if PostgreSQL client is installed
if ! command -v psql &> /dev/null; then
    echo "⚠️  PostgreSQL client (psql) not found"
    echo "Install with: sudo apt-get install postgresql-client (Linux)"
    echo "              brew install postgresql (macOS)"
    exit 1
fi

echo "Testing database connection..."
if psql "$DATABASE_URL" -c "SELECT 1" > /dev/null 2>&1; then
    echo "✅ Database connection successful"
else
    echo "❌ Cannot connect to database"
    echo "Check your DATABASE_URL in .env"
    exit 1
fi

echo ""
echo "Running migrations..."
echo "---------------------"

# Run migration
if psql "$DATABASE_URL" -f migrations/0001_initial_schema.sql > /dev/null 2>&1; then
    echo "✅ Migrations completed successfully"
else
    echo "⚠️  Migration completed with warnings (tables may already exist)"
fi

echo ""
echo "Verifying database schema..."
echo "-----------------------------"

# Check tables
echo ""
echo "Tables created:"
psql "$DATABASE_URL" -c "\dt" | head -20

echo ""
echo "✅ Database setup complete!"
echo ""
echo "Next steps:"
echo "1. Start the server: npm run dev"
echo "2. Test health endpoint: curl http://localhost:3001/health"
echo "3. Sign up a user and test the API"
echo ""
