# Use Bun for build speed
FROM oven/bun:1 AS base
WORKDIR /app

# Install dependencies first (cache layer)
COPY package.json bun.lock* ./
RUN bun install --frozen-lockfile

# Copy application code
COPY . .

# ── Build stage ──
FROM base AS build
# Build the PWA static files
RUN bun run build

# ── Production stage: serve static files with lightweight server ──
FROM oven/bun:1-slim AS production
WORKDIR /app

# Copy only the built output and package files needed for preview
COPY --from=build /app/dist ./dist
COPY --from=build /app/node_modules ./node_modules
COPY --from=build /app/package.json ./package.json
COPY --from=build /app/vite.config.ts ./vite.config.ts

EXPOSE 80

CMD ["bun", "run", "preview", "--host", "0.0.0.0", "--port", "80"]
