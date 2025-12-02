# Feature Specification: Render Backend Deployer Skill

**Feature Branch**: `001-render-backend-deployer`
**Created**: 2025-12-02
**Status**: Draft
**Input**: User description: "Create a reusable skill for deploying Python/FastAPI backends to Render.com with render.yaml Blueprint. The skill should guide through database creation (PostgreSQL, Redis), environment variable configuration, debugging common deployment errors (DATABASE_URL mismatches, CORS issues, middleware order), and frontend URL configuration. It should be based on the learnings from deploying the RAG chatbot backend that we just completed in this session."

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Initial Backend Deployment (Priority: P1)

A developer has completed a Python/FastAPI backend application locally and needs to deploy it to Render.com for production use. They want to use Render's Blueprint feature with a render.yaml file for infrastructure-as-code deployment.

**Why this priority**: This is the core functionality - without successful initial deployment, no other features matter. This represents the MVP that delivers immediate value.

**Independent Test**: Can be fully tested by deploying a simple FastAPI app with render.yaml to Render.com and verifying it's accessible via the provided URL.

**Acceptance Scenarios**:

1. **Given** a FastAPI backend with a render.yaml file at the repository root, **When** the developer follows the skill's guidance to create a web service on Render.com, **Then** the service deploys successfully and becomes accessible at the Render-provided URL
2. **Given** a deployed backend service, **When** the developer views the deployment logs, **Then** they see successful build completion, migrations running (if applicable), and the server starting
3. **Given** a render.yaml with Docker configuration, **When** the deployment runs, **Then** the Docker image builds successfully with correct build context and Dockerfile path

---

### User Story 2 - Database Setup and Configuration (Priority: P1)

A developer needs to set up PostgreSQL and Redis databases for their backend application and connect them properly with environment variables.

**Why this priority**: Most production backends require databases. Without proper database setup, the application won't function. This is part of the MVP.

**Independent Test**: Can be tested by creating PostgreSQL and Redis instances on Render, connecting them to a backend service, and verifying the application can read/write data successfully.

**Acceptance Scenarios**:

1. **Given** a backend requiring PostgreSQL, **When** the developer creates a PostgreSQL database on Render following the skill's guidance, **Then** the database is created with the correct name, region, and plan
2. **Given** a backend requiring Redis, **When** the developer creates a Redis (Key Value) instance on Render, **Then** the instance is created and provides an internal connection URL
3. **Given** created database instances, **When** the developer configures DATABASE_URL and REDIS_URL environment variables, **Then** the backend service can successfully connect to both databases
4. **Given** a database connection error, **When** the developer checks the environment variables, **Then** they can identify and fix mismatched variable names (e.g., DATABASE_URL vs NEON_DATABASE_URL)

---

### User Story 3 - Environment Variable Configuration (Priority: P1)

A developer needs to configure sensitive environment variables (API keys, database URLs, service URLs) for their backend application without hardcoding them.

**Why this priority**: Security and configuration management are critical for production deployments. This is essential for any real-world application.

**Independent Test**: Can be tested by adding environment variables through Render's UI and verifying the application can access them at runtime.

**Acceptance Scenarios**:

1. **Given** a backend requiring API keys (OpenAI, Qdrant, etc.), **When** the developer adds environment variables through Render's Environment tab, **Then** the variables are securely stored and available to the application
2. **Given** environment variables defined in render.yaml, **When** the service is created, **Then** non-secret variables are automatically configured from the Blueprint
3. **Given** secret environment variables not in render.yaml, **When** the developer manually adds them through the UI, **Then** they are securely stored and not exposed in logs
4. **Given** environment variable changes, **When** the developer saves changes, **Then** the service automatically redeploys with the new configuration

---

### User Story 4 - CORS Configuration for Frontend Integration (Priority: P2)

A developer needs to configure CORS (Cross-Origin Resource Sharing) to allow their frontend (hosted on GitHub Pages or another domain) to communicate with the backend API.

**Why this priority**: Essential for frontend-backend integration but can be configured after initial deployment. This enables the full-stack application.

**Independent Test**: Can be tested by making an API request from a frontend hosted on a different domain and verifying no CORS errors occur.

**Acceptance Scenarios**:

1. **Given** a FastAPI backend with CORS middleware, **When** the developer configures allowed_origins environment variable, **Then** the backend accepts requests from specified origins
2. **Given** a Pydantic settings configuration reading ALLOWED_ORIGINS, **When** the environment variable contains comma-separated URLs, **Then** the application correctly parses them into a list
3. **Given** CORS middleware and rate limiting middleware, **When** the developer configures middleware order, **Then** CORS middleware is added before rate limiting to ensure preflight requests get proper headers
4. **Given** a deployed backend, **When** a frontend makes a preflight OPTIONS request, **Then** the request succeeds with appropriate Access-Control headers

---

### User Story 5 - Deployment Error Debugging (Priority: P2)

A developer encounters deployment failures or runtime errors and needs to systematically debug and resolve them using deployment logs and error messages.

**Why this priority**: Deployment rarely works perfectly on the first try. Good debugging guidance significantly reduces time-to-resolution.

**Independent Test**: Can be tested by intentionally introducing common errors (wrong DATABASE_URL, missing dependencies, etc.) and using the skill's guidance to identify and fix them.

**Acceptance Scenarios**:

1. **Given** a deployment failure with "Can't load plugin: sqlalchemy.dialects:driver" error, **When** the developer checks environment variables, **Then** they identify the DATABASE_URL is missing or incorrectly named
2. **Given** a Pydantic validation error for allowed_origins, **When** the developer reviews the error logs, **Then** they identify type mismatch issues and fix the field annotation
3. **Given** build logs showing dependency installation errors, **When** the developer reviews requirements.txt, **Then** they identify missing or incompatible packages
4. **Given** health check failures in logs, **When** the developer checks service configuration, **Then** they verify the health check path matches the actual endpoint

---

### User Story 6 - Frontend URL Configuration (Priority: P3)

A developer needs to update their frontend application to use the correct backend API URL after deploying to Render.

**Why this priority**: Important for production but can be done after backend is deployed. This completes the deployment workflow.

**Independent Test**: Can be tested by updating frontend configuration files with the Render backend URL and verifying API calls succeed.

**Acceptance Scenarios**:

1. **Given** a deployed backend on Render with URL format `https://service-name-XXXX.onrender.com`, **When** the developer copies the service URL from Render dashboard, **Then** they can update frontend configuration files with the correct URL
2. **Given** frontend files referencing old/incorrect backend URLs, **When** the developer searches for backend URL references, **Then** they can identify all files needing updates (ChatWidget, API services, etc.)
3. **Given** updated frontend configuration, **When** the frontend is redeployed, **Then** API calls successfully reach the backend without CORS errors

---

### Edge Cases

- What happens when the render.yaml file has incorrect paths (wrong Dockerfile path, wrong root directory)?
- How does the system handle database creation when the database name already exists?
- What happens when environment variables contain special characters or spaces?
- How does the system behave when the free tier limits are exceeded (connection limits, memory limits)?
- What happens when the Docker build context and Dockerfile path combination is incorrect?
- How does the system handle migration failures during deployment?
- What happens when the backend URL changes after frontend deployment?
- How does the system handle middleware order issues beyond CORS/rate-limiting?

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: Skill MUST provide step-by-step guidance for creating a Render web service from a render.yaml Blueprint file
- **FR-002**: Skill MUST guide users through creating PostgreSQL databases on Render with correct configuration (name, user, region, plan)
- **FR-003**: Skill MUST guide users through creating Redis (Key Value) instances on Render with correct configuration
- **FR-004**: Skill MUST identify and explain common environment variable naming mismatches (DATABASE_URL vs NEON_DATABASE_URL)
- **FR-005**: Skill MUST provide guidance for configuring secure environment variables (API keys) through Render's UI
- **FR-006**: Skill MUST explain Pydantic Settings configuration patterns for environment variables
- **FR-007**: Skill MUST identify and fix CORS configuration issues including middleware order problems
- **FR-008**: Skill MUST diagnose common deployment errors from build and runtime logs
- **FR-009**: Skill MUST provide systematic debugging steps for SQLAlchemy database connection errors
- **FR-010**: Skill MUST guide users through updating frontend configuration files with correct backend URLs
- **FR-011**: Skill MUST explain Docker build context and Dockerfile path configuration for Render
- **FR-012**: Skill MUST validate that render.yaml configuration matches Render UI settings
- **FR-013**: Skill MUST provide guidance for reading and interpreting Render deployment logs
- **FR-014**: Skill MUST explain FastAPI middleware order and its impact on CORS and rate limiting
- **FR-015**: Skill MUST guide users through verifying successful deployment via health check endpoints

### Key Entities *(include if feature involves data)*

- **Render Service**: A web service instance on Render.com with configuration including name, runtime (Docker), region, environment variables, and deployment settings
- **Database Instance**: PostgreSQL or Redis database with connection URL, credentials, region, and plan tier
- **Environment Variable**: Key-value configuration pair with name, value, and visibility (secret/non-secret)
- **render.yaml Blueprint**: Infrastructure-as-code configuration file defining services, databases, and their relationships
- **Deployment Log**: Chronological record of build and runtime events including errors, warnings, and status messages
- **Frontend Configuration**: Files containing backend API URL references that need updating after backend deployment

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Developers can successfully deploy a FastAPI backend to Render.com within 15 minutes following the skill's guidance (from render.yaml to live service)
- **SC-002**: 90% of common deployment errors (DATABASE_URL mismatch, CORS issues, middleware order) are resolved within 10 minutes using the skill's debugging guidance
- **SC-003**: Database setup (PostgreSQL + Redis) is completed within 5 minutes with correct connection strings configured
- **SC-004**: CORS configuration issues are identified and resolved within 5 minutes, with frontend successfully communicating with backend
- **SC-005**: Environment variables are correctly configured on first attempt for at least 80% of deployments
- **SC-006**: Frontend URL updates are completed within 5 minutes across all configuration files
- **SC-007**: Deployment success rate improves to 90% on first attempt after following the skill's pre-deployment checklist
- **SC-008**: Time spent debugging deployment errors decreases by 70% compared to manual debugging without guidance
