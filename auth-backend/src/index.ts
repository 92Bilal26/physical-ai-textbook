import express, { Request, Response, NextFunction } from "express";
import cors from "cors";
import dotenv from "dotenv";
import { auth } from "./auth";
import { userProfileRoutes } from "./routes/userProfile";
import { setupUserProfileSchema } from "./db/userProfileSchema";
import { validateDatabaseConnection } from "./db";
import { toNodeHandler } from "better-auth/node";

dotenv.config();

const app = express();
const PORT = process.env.PORT || 3001;
const FRONTEND_URL = process.env.FRONTEND_URL || "http://localhost:3000";

// Parse FRONTEND_URL for CORS (can be comma-separated)
const allowedOrigins = FRONTEND_URL.split(",").map(url => url.trim());

// Middleware
app.use(express.json());
app.use(cors({
  origin: allowedOrigins,
  credentials: true,
}));

// Logging middleware
app.use((req: Request, _res: Response, next: NextFunction) => {
  console.log(`[${new Date().toISOString()}] ${req.method} ${req.path}`);
  next();
});

// Better Auth routes - use toNodeHandler for Express compatibility
app.all("/api/auth/*", toNodeHandler(auth));

// User profile routes
app.use("/api/users", userProfileRoutes);

// Health check
app.get("/health", (_req: Request, res: Response) => {
  res.json({ status: "ok", timestamp: new Date().toISOString() });
});

// Initialize database schema
async function initializeDatabase() {
  try {
    console.log("📡 Validating database connection...");
    const isConnected = await validateDatabaseConnection();

    if (!isConnected) {
      console.warn("⚠️  Database connection failed - some features may not work");
      return;
    }

    console.log("✅ Setting up user profile schema...");
    await setupUserProfileSchema();
    console.log("✅ Database schema initialized successfully");
  } catch (error) {
    // Log the error but don't fail - tables might already exist
    console.warn("⚠️  Database schema warning:", error);
    console.log("ℹ️  Continuing with existing schema...");
  }
}

// Start server
async function startServer() {
  try {
    console.log("ℹ️  Environment check:");
    console.log(`   - PORT: ${PORT}`);
    console.log(`   - FRONTEND_URL: ${FRONTEND_URL}`);
    console.log(`   - NODE_ENV: ${process.env.NODE_ENV}`);
    console.log(`   - BETTER_AUTH_SECRET: ${process.env.BETTER_AUTH_SECRET ? "✅ Set" : "❌ Not set"}`);
    console.log(`   - DATABASE_URL: ${process.env.DATABASE_URL ? "✅ Set" : "❌ Not set"}`);

    await initializeDatabase();

    const server = app.listen(PORT, () => {
      console.log(`\n🚀 Auth server running on http://localhost:${PORT}`);
      console.log(`📍 Frontend URL: ${FRONTEND_URL}`);
      console.log(`📚 API Routes:`);
      console.log(`   - POST /api/auth/sign-up`);
      console.log(`   - POST /api/auth/sign-in`);
      console.log(`   - POST /api/auth/sign-out`);
      console.log(`   - GET /api/auth/session`);
      console.log(`   - GET /api/users/profile`);
      console.log(`   - PUT /api/users/profile`);
      console.log(`   - GET /health`);
    });

    // Handle graceful shutdown
    process.on("SIGTERM", () => {
      console.log("SIGTERM received, shutting down gracefully");
      server.close(() => {
        console.log("Server closed");
        process.exit(0);
      });
    });
  } catch (error) {
    console.error("❌ Failed to start server:", error);
    process.exit(1);
  }
}

// Ensure console output is written immediately
process.stdout.write("Starting auth server...\n");
startServer().catch(error => {
  process.stderr.write(`Fatal error: ${JSON.stringify(error)}\n`);
  process.exit(1);
});
