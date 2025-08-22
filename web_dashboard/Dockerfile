# web_dashboard/Dockerfile
FROM node:18-alpine AS builder
WORKDIR /app
COPY package.json package-lock.json ./
RUN npm ci
COPY . .
RUN npm run build

FROM node:18-alpine AS runner
WORKDIR /app
# Install a simple static server to serve the compiled build
RUN npm install -g serve
COPY --from=builder /app/build /app/build

EXPOSE 3000
CMD ["serve", "-s", "build", "-l", "3000"]
