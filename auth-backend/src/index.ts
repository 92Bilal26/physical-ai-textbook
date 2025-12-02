import express, { Request, Response, NextFunction } from "express";
import cors from "cors";
import dotenv from "dotenv";
import { auth } from "./auth";
import { userProfileRoutes } from "./routes/userProfile";
import { setupUserProfileSchema } from "./db/userProfileSchema";

dotenv.config();

const app = express();
const PORT = process.env.PORT || 3001;
const FRONTEND_URL = process.env.FRONTEND_URL || "http://localhost:3000";

// Middleware
app.use(express.json());
app.use(cors({
  origin: FRONTEND_URL,
  credentials: true,
}));

// Logging middleware
app.use((req: Request, _res: Response, next: NextFunction) => {
  console.log(`[${new Date().toISOString()}] ${req.method} ${req.path}`);
  next();
});

// Better Auth routes
app.all("/api/auth/*", (req: Request, res: Response) => {
  // Using auth's handler method
  const handler = (auth as any).handler;
  if (handler) {
    return handler(req, res);
  }
  res.status(500).json({ error: "Auth handler not configured" });
});

// User profile routes
app.use("/api/users", userProfileRoutes);

// Health check
app.get("/health", (_req: Request, res: Response) => {
  res.json({ status: "ok", timestamp: new Date().toISOString() });
});

// Initialize database schema
async function initializeDatabase() {
  try {
    console.log("Setting up user profile schema...");
    await setupUserProfileSchema();
    console.log("Database schema initialized successfully");
  } catch (error) {
    // Log the error but don't fail - tables might already exist
    console.warn("Database schema warning:", error);
    console.log("Continuing with existing schema...");
  }
}

// Start server
async function startServer() {
  try {
    await initializeDatabase();
    app.listen(PORT, () => {
      console.log(`🚀 Auth server running on http://localhost:${PORT}`);
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
  } catch (error) {
    console.error("Failed to start server:", error);
    process.exit(1);
  }
}

startServer();
