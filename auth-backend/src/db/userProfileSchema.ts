import { db } from "./index";
import { userProfiles } from "./schema";
import { eq } from "drizzle-orm";
import fs from "fs";
import path from "path";
import postgres from "postgres";

export async function setupUserProfileSchema() {
  try {
    console.log("📋 Setting up database schema...");

    // Read and execute the migration file
    const migrationPath = path.join(process.cwd(), "migrations", "0001_initial_schema.sql");

    if (!fs.existsSync(migrationPath)) {
      console.warn("⚠️  Migration file not found, attempting to create tables with Drizzle ORM...");
      // Try to create tables using raw SQL as fallback
      const createTablesSQL = `
        CREATE TABLE IF NOT EXISTS users (
          id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
          name VARCHAR(255),
          email VARCHAR(255) NOT NULL UNIQUE,
          email_verified BOOLEAN DEFAULT false,
          password VARCHAR(255),
          image TEXT,
          created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
          updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
        );

        CREATE TABLE IF NOT EXISTS user_profiles (
          id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
          user_id UUID NOT NULL UNIQUE,
          software_dev_experience VARCHAR(50) DEFAULT 'beginner',
          robotics_background VARCHAR(50) DEFAULT 'none',
          programming_languages TEXT,
          preferred_language VARCHAR(10) DEFAULT 'en',
          content_preferences JSONB,
          created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
          updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
          FOREIGN KEY (user_id) REFERENCES users(id) ON DELETE CASCADE
        );

        CREATE TABLE IF NOT EXISTS sessions (
          id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
          user_id UUID NOT NULL,
          token VARCHAR(255) NOT NULL UNIQUE,
          expires_at TIMESTAMP NOT NULL,
          created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
          FOREIGN KEY (user_id) REFERENCES users(id) ON DELETE CASCADE
        );

        CREATE TABLE IF NOT EXISTS verification_tokens (
          id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
          user_id UUID NOT NULL,
          token VARCHAR(255) NOT NULL UNIQUE,
          type VARCHAR(50) NOT NULL,
          expires_at TIMESTAMP NOT NULL,
          created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
          FOREIGN KEY (user_id) REFERENCES users(id) ON DELETE CASCADE
        );

        CREATE TABLE IF NOT EXISTS accounts (
          id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
          user_id UUID NOT NULL,
          provider VARCHAR(50) NOT NULL,
          provider_account_id VARCHAR(255) NOT NULL,
          access_token TEXT,
          refresh_token TEXT,
          expires_at TIMESTAMP,
          created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
          updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
          FOREIGN KEY (user_id) REFERENCES users(id) ON DELETE CASCADE
        );

        CREATE INDEX IF NOT EXISTS idx_users_email ON users(email);
        CREATE INDEX IF NOT EXISTS idx_user_profiles_user_id ON user_profiles(user_id);
        CREATE INDEX IF NOT EXISTS idx_sessions_user_id ON sessions(user_id);
        CREATE INDEX IF NOT EXISTS idx_sessions_token ON sessions(token);
        CREATE INDEX IF NOT EXISTS idx_verification_tokens_user_id ON verification_tokens(user_id);
        CREATE INDEX IF NOT EXISTS idx_verification_tokens_token ON verification_tokens(token);
        CREATE INDEX IF NOT EXISTS idx_accounts_user_id ON accounts(user_id);
        CREATE INDEX IF NOT EXISTS idx_accounts_provider ON accounts(provider);
      `;

      const sql = postgres(process.env.DATABASE_URL || "");
      await sql.unsafe(createTablesSQL);
      await sql.end();
      console.log("✅ Database schema created successfully");
      return;
    }

    // Read migration SQL
    const migrationSQL = fs.readFileSync(migrationPath, "utf-8");

    // Execute migration
    const sql = postgres(process.env.DATABASE_URL || "");
    await sql.unsafe(migrationSQL);
    await sql.end();

    console.log("✅ Database schema initialized successfully");
  } catch (error) {
    console.warn("⚠️  Database schema initialization warning:", error);
    console.log("ℹ️  Continuing... Tables might already exist");
  }
}

export async function createUserProfile(userId: string, data: Partial<typeof userProfiles.$inferInsert>) {
  try {
    const result = await db
      .insert(userProfiles)
      .values({
        userId,
        ...data,
      })
      .returning();
    return result[0];
  } catch (error) {
    console.error("Error creating user profile:", error);
    throw error;
  }
}

export async function getUserProfile(userId: string) {
  try {
    const profile = await db
      .select()
      .from(userProfiles)
      .where(eq(userProfiles.userId, userId))
      .limit(1);
    return profile[0] || null;
  } catch (error) {
    console.error("Error fetching user profile:", error);
    throw error;
  }
}

export async function updateUserProfile(userId: string, data: Partial<typeof userProfiles.$inferInsert>) {
  try {
    const result = await db
      .update(userProfiles)
      .set({
        ...data,
        updatedAt: new Date(),
      })
      .where(eq(userProfiles.userId, userId))
      .returning();
    return result[0];
  } catch (error) {
    console.error("Error updating user profile:", error);
    throw error;
  }
}

export async function deleteUserProfile(userId: string) {
  try {
    await db
      .delete(userProfiles)
      .where(eq(userProfiles.userId, userId));
  } catch (error) {
    console.error("Error deleting user profile:", error);
    throw error;
  }
}
