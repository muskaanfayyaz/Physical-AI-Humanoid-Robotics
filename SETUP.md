# Physical AI Textbook - Setup and Run Instructions

## Quick Start

Follow these steps to run the textbook locally:

### 1. Install Node.js

Make sure you have Node.js 18.0 or higher installed.

Check your version:
```bash
node --version
```

If you need to install Node.js, download it from: https://nodejs.org/

### 2. Install Dependencies

Navigate to the textbook directory and install dependencies:

```bash
cd /mnt/d/physical-ai-textbook
npm install
```

### 3. Run the Development Server

Start the local development server:

```bash
npm start
```

The textbook will open in your browser at `http://localhost:3000`

### 4. Build for Production

To create a production build:

```bash
npm run build
```

The static files will be generated in the `build/` directory.

### 5. Serve Production Build

To preview the production build locally:

```bash
npm run serve
```

## Project Structure

```
physical-ai-textbook/
├── docs/                          # All markdown content
│   ├── introduction.md            # Landing page
│   └── chapters/                  # Textbook chapters
│       ├── chapter-01-*.md
│       ├── chapter-02-*.md
│       └── ...
├── src/
│   └── css/
│       └── custom.css             # Custom styling
├── static/                        # Static assets
│   └── img/
├── docusaurus.config.js           # Main configuration
├── sidebars.js                    # Sidebar navigation
└── package.json                   # Dependencies
```

## Customization

### Update GitHub Repository

Edit `docusaurus.config.js`:

```javascript
url: 'https://yourusername.github.io',
baseUrl: '/physical-ai-textbook/',
organizationName: 'yourusername',
projectName: 'physical-ai-textbook',
```

Replace `yourusername` with your actual GitHub username.

### Customize Theme

Edit `src/css/custom.css` to change colors, fonts, and styling.

### Add Logo

Place your logo image in `static/img/logo.svg` (or .png)

### Update Footer

Edit the footer section in `docusaurus.config.js` under `themeConfig.footer`.

## Deployment

### Deploy to GitHub Pages

1. Update `docusaurus.config.js` with your GitHub info
2. Build the site:
   ```bash
   npm run build
   ```
3. Deploy:
   ```bash
   GIT_USER=<your-github-username> npm run deploy
   ```

### Deploy to Vercel

1. Push your repository to GitHub
2. Connect your repo to Vercel
3. Vercel will automatically detect Docusaurus and deploy

### Deploy to Netlify

1. Push your repository to GitHub
2. Connect your repo to Netlify
3. Build command: `npm run build`
4. Publish directory: `build`

## Troubleshooting

### Port Already in Use

If port 3000 is already in use, specify a different port:

```bash
npm start -- --port 3001
```

### Module Not Found

Delete `node_modules` and reinstall:

```bash
rm -rf node_modules package-lock.json
npm install
```

### Build Errors

Clear the cache and rebuild:

```bash
npm run clear
npm run build
```

## Features

- 18 comprehensive chapters
- 5 detailed appendices
- Organized weekly sidebar navigation
- Dark/light mode support
- Mobile-responsive design
- Search functionality
- ASCII diagram rendering
- Code syntax highlighting

## Support

For issues or questions:
- Check the [Docusaurus documentation](https://docusaurus.io)
- Review the README.md file
- Contact the Panaversity team

## License

Educational use for Panaversity Physical AI & Humanoid Robotics course.
