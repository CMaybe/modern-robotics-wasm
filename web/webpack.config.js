const path = require("path");
const HtmlWebpackPlugin = require("html-webpack-plugin");

module.exports = {
  entry: "./src/index.tsx",
  output: {
    path: path.resolve(__dirname, "dist"),
    filename: "bundle.[contenthash].js",
    publicPath: "/",
    clean: true,
  },
  resolve: {
    extensions: [".ts", ".tsx", ".js"],
  },
  module: {
    rules: [
      {
        test: /\.(ts|tsx)$/,
        use: "babel-loader",
        exclude: /node_modules/,
      },
      {
        test: /\.css$/,
        use: ["style-loader", "css-loader"],
      },
    ],
  },
  plugins: [
    new HtmlWebpackPlugin({
      template: "./public/index.html",
    }),
  ],
  devServer: {
    // The Emscripten glue and the .wasm binary live in public/wasm and are loaded
    // at runtime, so they are served statically rather than bundled.
    static: {
      directory: path.resolve(__dirname, "public"),
    },
    host: "0.0.0.0",
    port: 3000,
    hot: true,
    historyApiFallback: true,
    // Required so the container's dev server is reachable from the host browser.
    allowedHosts: "all",
  },
  performance: {
    hints: false,
  },
  mode: "development",
};
