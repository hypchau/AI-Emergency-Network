#!/usr/bin/env python3
"""
Generate the custom OMNeT++ icons used by the AI Emergency Network simulation.

The icons are drawn programmatically (no third-party artwork), so they can be
regenerated or restyled at will:

    python tools/make_icons.py

Output: images/aien/*.png  (referenced in NED/ini as i=aien/<name>)
Requires: Pillow (pip install pillow)
"""
import math
import os

from PIL import Image, ImageDraw

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "..", "images", "aien")

SS = 4  # supersampling factor for smooth edges


def canvas(size):
    return Image.new("RGBA", (size * SS, size * SS), (0, 0, 0, 0))


def finish(img, size, name):
    img = img.resize((size, size), Image.LANCZOS)
    os.makedirs(OUT, exist_ok=True)
    img.save(os.path.join(OUT, name + ".png"))


def drone(name, ring, size=48):
    """Top view of a quadcopter; `ring` colours the propeller guards (state)."""
    img = canvas(size)
    d = ImageDraw.Draw(img)
    s = size * SS
    c = s / 2
    arm = s * 0.30
    rotor = s * 0.17
    body = (45, 52, 64, 255)
    # arms
    for k in range(4):
        a = math.radians(45 + 90 * k)
        x, y = c + arm * math.cos(a), c + arm * math.sin(a)
        d.line([(c, c), (x, y)], fill=body, width=int(s * 0.07))
    # rotors
    for k in range(4):
        a = math.radians(45 + 90 * k)
        x, y = c + arm * math.cos(a), c + arm * math.sin(a)
        d.ellipse([x - rotor, y - rotor, x + rotor, y + rotor],
                  fill=ring + (110,), outline=ring + (255,), width=int(s * 0.035))
        d.ellipse([x - s * 0.025, y - s * 0.025, x + s * 0.025, y + s * 0.025], fill=body)
    # body with the 4G payload (small white antenna mark)
    b = s * 0.13
    d.rounded_rectangle([c - b, c - b, c + b, c + b], radius=s * 0.04, fill=body)
    d.ellipse([c - s * 0.045, c - s * 0.045, c + s * 0.045, c + s * 0.045], fill=(255, 255, 255, 255))
    finish(img, size, name)


def truck(name="truck", size=64):
    """Side view of an off-road truck carrying a mast/antenna (the 'hive')."""
    img = canvas(size)
    d = ImageDraw.Draw(img)
    s = size * SS
    red = (200, 40, 40, 255)
    dark = (40, 40, 40, 255)
    grey = (120, 120, 120, 255)
    # chassis + box
    d.rounded_rectangle([s * 0.08, s * 0.50, s * 0.66, s * 0.78], radius=s * 0.03, fill=(235, 235, 235, 255),
                        outline=dark, width=int(s * 0.02))
    # cab
    d.rounded_rectangle([s * 0.66, s * 0.56, s * 0.92, s * 0.78], radius=s * 0.04, fill=red,
                        outline=dark, width=int(s * 0.02))
    d.rectangle([s * 0.72, s * 0.60, s * 0.86, s * 0.67], fill=(170, 210, 240, 255))
    # red cross on the box
    d.rectangle([s * 0.33, s * 0.55, s * 0.41, s * 0.73], fill=red)
    d.rectangle([s * 0.28, s * 0.60, s * 0.46, s * 0.68], fill=red)
    # wheels (big, off-road)
    for wx in (0.22, 0.52, 0.80):
        r = s * 0.085
        cx, cy = s * wx, s * 0.80
        d.ellipse([cx - r, cy - r, cx + r, cy + r], fill=dark)
        d.ellipse([cx - r * 0.45, cy - r * 0.45, cx + r * 0.45, cy + r * 0.45], fill=grey)
    # mast + antenna with radio waves
    mx = s * 0.18
    d.line([(mx, s * 0.50), (mx, s * 0.12)], fill=dark, width=int(s * 0.03))
    d.polygon([(mx - s * 0.05, s * 0.20), (mx + s * 0.05, s * 0.20), (mx, s * 0.08)], fill=red)
    for k, rr in enumerate((0.10, 0.17)):
        box = [mx - s * rr, s * 0.12 - s * rr, mx + s * rr, s * 0.12 + s * rr]
        d.arc(box, start=-60, end=60, fill=(30, 110, 200, 255), width=int(s * 0.022))
        d.arc(box, start=120, end=240, fill=(30, 110, 200, 255), width=int(s * 0.022))
    finish(img, size, name)


def person(name, colour, sos, size=32):
    """Simple pictogram of a person; red with SOS bubble when not covered."""
    img = canvas(size)
    d = ImageDraw.Draw(img)
    s = size * SS
    c = s / 2
    d.ellipse([c - s * 0.11, s * 0.10, c + s * 0.11, s * 0.32], fill=colour)
    d.rounded_rectangle([c - s * 0.16, s * 0.34, c + s * 0.16, s * 0.66], radius=s * 0.06, fill=colour)
    d.rectangle([c - s * 0.13, s * 0.62, c - s * 0.03, s * 0.92], fill=colour)
    d.rectangle([c + s * 0.03, s * 0.62, c + s * 0.13, s * 0.92], fill=colour)
    if sos:
        # small exclamation badge
        d.ellipse([s * 0.62, s * 0.02, s * 0.98, s * 0.38], fill=(255, 210, 0, 255), outline=(60, 60, 60, 255),
                  width=int(s * 0.02))
        d.rectangle([s * 0.775, s * 0.08, s * 0.825, s * 0.24], fill=(60, 60, 60, 255))
        d.rectangle([s * 0.775, s * 0.27, s * 0.825, s * 0.32], fill=(60, 60, 60, 255))
    else:
        # small signal bars (connected)
        for k in range(3):
            h = s * (0.08 + 0.07 * k)
            x0 = s * (0.70 + 0.09 * k)
            d.rectangle([x0, s * 0.30 - h, x0 + s * 0.06, s * 0.30], fill=(20, 120, 40, 255))
    finish(img, size, name)


def post(name="post", size=24):
    """Diamond marker for a planned relay post."""
    img = canvas(size)
    d = ImageDraw.Draw(img)
    s = size * SS
    c = s / 2
    r = s * 0.42
    d.polygon([(c, c - r), (c + r, c), (c, c + r), (c - r, c)], fill=(255, 255, 255, 200),
              outline=(30, 110, 200, 255))
    d.ellipse([c - s * 0.1, c - s * 0.1, c + s * 0.1, c + s * 0.1], fill=(30, 110, 200, 255))
    finish(img, size, name)


if __name__ == "__main__":
    drone("drone_parked", (140, 140, 140))     # at the hive, ready
    drone("drone_outbound", (30, 110, 200))    # flying to its post
    drone("drone_onstation", (20, 160, 60))    # on post, serving users
    drone("drone_returning", (240, 140, 20))   # going back for a battery swap
    drone("drone_swapping", (150, 90, 200))    # battery being swapped / waiting for a charged pack
    drone("drone_failed", (210, 30, 30))       # failure injected
    truck()
    person("victim_sos", (210, 40, 40, 255), True)
    person("victim_ok", (30, 140, 60, 255), False)
    post()
    print("icons written to", os.path.normpath(OUT))
