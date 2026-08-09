# Bayou tutorial

A self-contained walkthrough of how sensor data gets from the receiver in
this repo to a chart on a Bayou feed — and how a short Python script can
play the sensor's role.

## Files

| File | What it is |
|---|---|
| `index.html` | The tutorial page. Open it directly in a browser or serve it. |
| `example_temp_poster.py` | Runnable Python script the tutorial references — posts random temperatures to a Bayou feed. |

## Viewing

**Locally**, just open `index.html` in a browser — no build step, no
external assets. All styles are inline.

**Via GitHub Pages**, whatever GitHub Pages setup this repo uses will serve
the folder at:

    https://<owner>.github.io/<repo>/bayou-tutorial/

Every asset the page references (the Python script, no images, no CDN CSS)
is relative to that folder, so it works under any URL prefix.

## Portability into Bayou

The page is deliberately self-contained so it can later be lifted into the
Bayou codebase (`~/gitwork/bayou`) with minimal editing:

- No external CSS, JS, fonts, or images.
- Semantic HTML with an inline `<style>` block using CSS variables.
- Uses `prefers-color-scheme: dark` so it looks reasonable against either
  a light or dark host theme.
- Pug conversion is mechanical if that's the target — the structure is
  flat sections with headings, paragraphs, `<pre>`, and one `<table>`.

## Updating

- **Firmware body changed?** The JSON POST body quoted in section 2 is
  from `examples/v3-ultrasonic/companion_sensor_receiver/main.cpp`, in
  `postToBayou()`. If the fields there change, sync the block here.
- **Bayou fields changed?** Section 4's table mirrors the "Recognized
  fields" table in the Bayou repo's `docs/API.md`. Regenerate from there
  when Bayou's schema gains a new column.
