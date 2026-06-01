# Image-Conditioned Generation

Articraft can use a local reference image alongside a text prompt. The image gives the model visual context, while the prompt should still describe the desired object, articulation, moving parts, and any constraints that matter.

## Generate from an Image

Use `--image` with `articraft generate`:

```bash
uv run articraft generate \
  --image path/to/reference.png \
  "Create an articulated 3D object based on this reference image."
```

You can combine image input with the normal generation controls:

```bash
uv run articraft generate \
  --image path/to/reference.png \
  --max-cost-usd 1.5 \
  "Create an articulated desk fan based on this reference image, with a tilting head and spinning blades."
```

If you override the model and Articraft cannot infer the provider from the model ID, pass `--provider` explicitly.

## Generate Directly into a Dataset Category

Use `--image` with `articraft dataset run`:

```bash
uv run articraft dataset run \
  --category-slug desk-lamps \
  --image path/to/reference.png \
  "Create an articulated desk lamp based on this reference image."
```

## Edit with a Reference Image

Image input is also supported for copy edits with `articraft fork`:

```bash
uv run articraft fork data/records/<record_id> \
  --image path/to/reference.png \
  "Update the object to match the latch shape in the reference image."
```

The forked record keeps the parent unchanged and stores the edit as a new child record.

## Create an Image-Backed Draft

Use `articraft draft --image` when you want to store a prompt and reference image without running generation yet:

```bash
uv run articraft draft \
  --image path/to/reference.png \
  "Create an articulated folding chair based on this reference image."
```

## Supported Image Formats

Supported formats depend on the provider:

| Provider | Supported formats |
| --- | --- |
| `openai` | PNG, JPEG, WebP, GIF |
| `anthropic` | PNG, JPEG, WebP, GIF |
| `openrouter` | PNG, JPEG, WebP, GIF |
| `gemini` | PNG, JPEG, WebP, HEIC, HEIF |
| `deepseek` | Not supported (no vision capability) |

Images must be local files. Gemini image inputs must stay under the inline request limit; other providers reject images larger than 50 MB.

## Batch Limitation

Tracked batch CSV generation does not currently support image inputs. The batch CSV v1 format rejects an `image_path` column, so use `articraft dataset run` for image-conditioned dataset records.
