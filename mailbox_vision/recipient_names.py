"""Shared recipient normalization and conservative database matching."""

import unicodedata
from difflib import SequenceMatcher


def normalize_name(text):
    text = unicodedata.normalize("NFKD", text).casefold()
    return "".join(c for c in text if c.isalnum() and not unicodedata.combining(c))


def match_recipient(text, recipients, *, min_score=0.6, min_margin=0.08):
    """Return a canonical database name or None when the result is weak/tied."""
    normalized = normalize_name(text)
    if not normalized or not recipients:
        return None, 0.0
    scores = sorted(
        (
            (SequenceMatcher(None, normalized, normalize_name(name)).ratio(), name)
            for name in recipients
        ),
        reverse=True,
    )
    score, name = scores[0]
    second = scores[1][0] if len(scores) > 1 else 0.0
    if score < min_score or score - second < min_margin:
        return None, score
    return name, score
