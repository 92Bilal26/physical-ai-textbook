# Physical AI Textbook - Auth Backend

Better Auth integration for the Physical AI Textbook with user authentication, profile management, and personalization features.

## Features

- **Email/Password Authentication**: Sign up and sign in with email and password
- **Social Authentication**: OAuth support for GitHub and Google
- **User Profiles**: Collect and manage user background information
- **Personalization**: Store user preferences for content recommendations
- **Session Management**: Secure session handling with JWT tokens

## Technology Stack

- **Runtime**: Node.js 18+
- **Framework**: Express.js
- **Authentication**: Better Auth
- **Database**: PostgreSQL with Drizzle ORM
- **Language**: TypeScript

## Setup

### 1. Install Dependencies

```bash
npm install
```

### 2. Configure Database

Create a PostgreSQL database and copy `.env.example` to `.env`:

```bash
cp .env.example .env
```

Update the `DATABASE_URL` with your PostgreSQL connection string:

```
DATABASE_URL=postgresql://username:password@localhost:5432/physical_ai_auth
```

### 3. Generate Database Migrations

```bash
npx drizzle-kit generate
```

### 4. Run Migrations

```bash
npx drizzle-kit push
```

### 5. Environment Variables

Required environment variables:

- `DATABASE_URL`: PostgreSQL connection string
- `BETTER_AUTH_SECRET`: Secret key for Better Auth (generate a random string)
- `FRONTEND_URL`: URL of the frontend application (default: http://localhost:3000)
- `PORT`: Server port (default: 3001)

Optional OAuth providers:

- `GITHUB_CLIENT_ID` and `GITHUB_CLIENT_SECRET`
- `GOOGLE_CLIENT_ID` and `GOOGLE_CLIENT_SECRET`

## API Endpoints

### Authentication

- `POST /api/auth/sign-up` - User registration
- `POST /api/auth/sign-in` - User login
- `POST /api/auth/sign-out` - User logout
- `GET /api/auth/session` - Get current session

### User Profile

- `GET /api/users/profile` - Get user profile (requires auth)
- `PUT /api/users/profile` - Update user profile (requires auth)
- `POST /api/users/profile/init` - Initialize profile on signup (requires auth)

### Health Check

- `GET /health` - Server health check

## User Profile Fields

When creating or updating a profile, include:

- `softwareDevelopmentExperience`: "beginner" | "intermediate" | "advanced"
- `roboticsHardwareBackground`: "none" | "basic" | "experienced"
- `programmingLanguages`: array of language strings
- `preferredLanguage`: language code (e.g., "en", "fr")
- `contentPreferences`: object with UI/content preferences

## Development

### Build

```bash
npm run build
```

### Development Server

```bash
npm run dev
```

### Run Production

```bash
npm start
```

### Linting

```bash
npm run lint
```

### Format Code

```bash
npm run format
```

## Database Schema

### Users

Core user data with email authentication.

### User Profiles

User background information for content personalization:
- Software development experience level
- Robotics/hardware background
- Programming language proficiency
- Content preferences

### Sessions

Session management for authenticated requests.

### Accounts

OAuth provider account connections.

### Verification Tokens

Email verification and password reset tokens.

## Error Handling

The API returns standard HTTP status codes:

- `200` - Success
- `201` - Created
- `400` - Bad Request
- `401` - Unauthorized
- `404` - Not Found
- `500` - Internal Server Error

## Security Considerations

1. **HTTPS**: Use HTTPS in production
2. **CORS**: Configure `FRONTEND_URL` for your frontend domain
3. **Secrets**: Use strong `BETTER_AUTH_SECRET` in production
4. **Database**: Use strong passwords and connection encryption
5. **Tokens**: Sessions expire after 7 days (configurable)
6. **Validation**: All input is validated before database operations

## Contributing

Follow the existing code style and add tests for new features.

## License

MIT
