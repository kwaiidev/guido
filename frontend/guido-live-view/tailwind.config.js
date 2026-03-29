/** @type {import('tailwindcss').Config} */
export default {
  content: ['./index.html', './src/**/*.{js,jsx}'],
  theme: {
    extend: {
      colors: {
        paper: '#f2efe8',
        ink: '#11140f',
        line: '#9fa48f',
        accent: '#96ff62',
        mist: '#d8d5cb',
      },
      fontFamily: {
        sans: ['"Space Grotesk"', '"Helvetica Neue"', 'sans-serif'],
        mono: ['"IBM Plex Mono"', '"SFMono-Regular"', 'monospace'],
      },
      boxShadow: {
        glow: '0 0 0 1px rgba(150, 255, 98, 0.24), 0 18px 40px rgba(17, 20, 15, 0.08)',
      },
      keyframes: {
        pulseDot: {
          '0%, 100%': { transform: 'scale(1)', opacity: '0.4' },
          '50%': { transform: 'scale(1.9)', opacity: '0.95' },
        },
        driftIn: {
          '0%': { opacity: '0', transform: 'translateY(16px)' },
          '100%': { opacity: '1', transform: 'translateY(0)' },
        },
      },
      animation: {
        pulseDot: 'pulseDot 1.8s ease-in-out infinite',
        driftIn: 'driftIn 600ms ease-out both',
      },
    },
  },
  plugins: [],
};
