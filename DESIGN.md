# Wobot Design System

## Direction

Electronics bench in daylight: a matte, precise instrument panel with one
olive status lamp. The product uses a restrained light theme so warnings,
progress, and the selected firmware target carry meaning immediately.

## Color

Use OKLCH values directly. Color is functional and occupies less than 10% of
the interface.

```css
:root {
  --bg: oklch(1 0 0);
  --surface: oklch(0.96 0.008 110);
  --ink: oklch(0.20 0.015 110);
  --muted: oklch(0.46 0.018 110);
  --primary: oklch(0.52 0.105 110);
  --accent: oklch(0.30 0.10 245);
  --danger: oklch(0.48 0.17 28);
  --line: oklch(0.87 0.012 110);
}
```

White text is used on primary, accent, and danger fills. Status always pairs
color with a word and, when useful, an icon.

## Typography

Use the device system sans stack. Labels and controls use 0.875–1rem; body
copy uses 1rem with 1.55 line height; the page heading uses 1.75rem. Technical
identifiers and versions use the system monospace stack. Keep prose under 70ch.

## Layout

Use a single centered column, at most 680px wide, with 24px desktop and 16px
mobile gutters. The status summary comes first, followed by target/file input,
the primary action, progress, and recovery guidance. Spacing follows an 8px
base rhythm.

## Components

- Buttons have a consistent 10px radius, clear focus ring, and at least 44px
  touch height. Only the update action is filled.
- Inputs use a native control wherever possible and expose their label outside
  the control.
- Progress uses a native progress element plus written and total byte counts.
- Alerts use a full tinted background and icon/text label, never color alone.
- Destructive or rebooting actions state their consequence in the button copy.

## Motion

Only state feedback moves, for 150–200ms. Respect `prefers-reduced-motion` and
do not animate page entry, progress layout, or decorative elements.

## Responsive and Accessibility

Meet WCAG 2.2 AA, preserve visible keyboard focus, use semantic HTML, and keep
the complete update flow usable at 320px width. Disable submission while an
upload is active and expose status changes through an ARIA live region.
