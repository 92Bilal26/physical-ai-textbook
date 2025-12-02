#!/bin/bash

# Better Auth - Quick Start Script for Mac/Linux

echo ""
echo "╔════════════════════════════════════════════════════════════╗"
echo "║         Better Auth - Local Development Starter            ║"
echo "║          Physical AI Textbook Authentication               ║"
echo "╚════════════════════════════════════════════════════════════╝"
echo ""

# Check if Docker is running
if ! command -v docker &> /dev/null; then
    echo "❌ Docker is not installed!"
    echo "Please install Docker from: https://www.docker.com/products/docker-desktop"
    exit 1
fi

# Check if PostgreSQL container exists
if docker ps -a | grep -q "physical-ai-auth-db"; then
    echo "📦 PostgreSQL container found"

    # Check if it's running
    if docker ps | grep -q "physical-ai-auth-db"; then
        echo "✅ PostgreSQL is already running"
    else
        echo "Starting PostgreSQL container..."
        docker start physical-ai-auth-db
        sleep 3
        echo "✅ PostgreSQL started!"
    fi
else
    echo "⚠️  PostgreSQL container not found."
    echo ""
    echo "Starting PostgreSQL in Docker..."
    echo ""

    docker run --name physical-ai-auth-db \
      -e POSTGRES_USER=authuser \
      -e POSTGRES_PASSWORD=authpass123 \
      -e POSTGRES_DB=physical_ai_auth \
      -p 5432:5432 \
      -d postgres:16-alpine

    echo ""
    echo "✅ PostgreSQL started!"
    echo ""
    echo "Waiting 5 seconds for database to initialize..."
    sleep 5
fi

echo ""
echo "📦 Installing dependencies..."
npm install
echo ""

echo ""
echo "🚀 Starting Better Auth Backend..."
echo ""
echo "    Backend URL: http://localhost:3001"
echo "    Test UI: Open test-client.html in your browser"
echo ""
echo "📖 For detailed guide, see: LOCAL_SETUP_GUIDE.md"
echo ""

npm run dev
