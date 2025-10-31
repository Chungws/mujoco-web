import type { NextConfig } from "next";

const nextConfig: NextConfig = {
  // Enable Turbopack with custom configuration
  turbopack: {},
  webpack: (config, { isServer }) => {
    // Don't resolve 'module' and 'fs' on client side
    if (!isServer) {
      config.resolve.fallback = {
        ...config.resolve.fallback,
        module: false,
        fs: false,
      };
    }
    return config;
  },
};

export default nextConfig;
