# Product

## Register

product

## Users

The primary user is the robot's owner and developer, working locally beside the
robot from a laptop or phone. They need to install firmware, understand device
state, and recover safely when an update is interrupted.

## Product Purpose

Wobot is a software platform for a four-wheel mecanum robot. Its maintenance
surface provides safe firmware updates and diagnostics for both the ESP32 brain
and the Arduino motor controller. Success means routine releases no longer need
a USB cable, while the first installation and emergency recovery remain clear.

## Brand Personality

Technical, calm, and unambiguous. The interface should feel like a trustworthy
instrument: concise status, explicit target selection, and visible progress.

## Anti-references

Avoid toy-like, gaming, neon, or marketing-dashboard styling. Avoid decorative
animation, hidden destructive actions, vague status messages, and controls that
make an update look less consequential than it is.

## Design Principles

1. Safety state is always visible before an update starts.
2. Explain the next physical action in plain language.
3. Progressive disclosure keeps the normal update path short while retaining
   useful technical detail for diagnosis.
4. Never imply success until the target has validated the written image.
5. Recovery instructions remain available when a normal path fails.

## Accessibility & Inclusion

Meet WCAG 2.2 AA contrast and interaction requirements. Work comfortably on a
phone and desktop. Respect reduced-motion preferences, preserve keyboard
operation, and never communicate status through color alone.
