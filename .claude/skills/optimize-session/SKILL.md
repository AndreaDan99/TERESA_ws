---
name: optimize-session
description: Suggerisce il modello giusto (Haiku/Sonnet/Opus) e gestisce il contesto per ridurre i token. Invoca prima di iniziare una task nuova.
disable-model-invocation: true
---

## Contesto sessione corrente
- Branch: !`git branch --show-current 2>/dev/null || echo "n/a"`
- Ultimi commit: !`git log --oneline -3 2>/dev/null || echo "n/a"`

Analizza la prossima task dell'utente e ottimizza la sessione in 3 passi. Rispondi in massimo 6 righe totali.

## Passo 1 — Classifica la task

| Tipo | Esempi | Modello |
|------|--------|---------|
| **Semplice** | git status, leggere un file, cambiare un parametro, domanda diretta | `haiku` |
| **Media** | modifiche multi-file, creare package, update LaTeX, discussione architetturale | `sonnet` |
| **Complessa** | debugging profondo su sistemi sconosciuti, progettazione da zero, quando Sonnet non risolve | `opus` |

## Passo 2 — Modello

Scrivi: **Modello: [nome]** — [motivo in 5 parole max]

Modelli:
- `claude-haiku-4-5-20251001` (~20x più economico)
- `claude-sonnet-4-6` (default)
- `claude-opus-4-6` (massima capacità)

## Passo 3 — Contesto

- Task non correlata alla precedente → "Digita `/clear`"
- Task correlata + conversazione lunga → "Digita `/compact`"
- Altrimenti → "Contesto ok"

## Regole rapide
- Dai sempre il path diretto invece di chiedere esplorazioni larghe
- Per file grandi usa `offset` + `limit`
- Usa Grep/Glob invece di Read quando cerchi qualcosa di specifico
