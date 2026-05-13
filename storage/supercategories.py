from __future__ import annotations

from dataclasses import dataclass

from storage.identifiers import validate_category_slug, validate_supercategory_slug
from storage.models import SupercategoryEntry, SupercategoryManifest
from storage.repo import StorageRepo


def _normalized_category_slugs(category_slugs: list[str]) -> list[str]:
    normalized: list[str] = []
    seen: set[str] = set()
    for value in category_slugs:
        if not str(value or "").strip():
            continue
        slug = validate_category_slug(value)
        if slug in seen:
            continue
        seen.add(slug)
        normalized.append(slug)
    return normalized


@dataclass(slots=True)
class SupercategoryStore:
    repo: StorageRepo

    def load_manifest(self) -> SupercategoryManifest | None:
        path = self.repo.layout.supercategories_path
        data = self.repo.read_json(path)
        if not isinstance(data, dict):
            return None
        entries = [
            SupercategoryEntry(
                slug=str(entry.get("slug", "")),
                title=str(entry.get("title", "")),
                description=str(entry.get("description", "")),
                category_slugs=[str(s) for s in entry.get("category_slugs", [])],
            )
            for entry in data.get("supercategories", [])
            if isinstance(entry, dict)
        ]
        return SupercategoryManifest(
            schema_version=int(data.get("schema_version", 1)),
            supercategories=entries,
        )

    def load_manifest_or_default(self) -> SupercategoryManifest:
        return self.load_manifest() or SupercategoryManifest(schema_version=1)

    def save(self, manifest: SupercategoryManifest) -> None:
        path = self.repo.layout.supercategories_path
        self.repo.write_json(path, manifest.to_dict())

    def load_entry(self, supercategory_slug: str) -> SupercategoryEntry | None:
        supercategory_slug = validate_supercategory_slug(supercategory_slug)
        manifest = self.load_manifest()
        if manifest is None:
            return None
        for entry in manifest.supercategories:
            if entry.slug == supercategory_slug:
                return entry
        return None

    def save_entry(self, entry: SupercategoryEntry) -> tuple[SupercategoryEntry, bool]:
        manifest = self.load_manifest_or_default()
        normalized_entry = SupercategoryEntry(
            slug=validate_supercategory_slug(entry.slug),
            title=str(entry.title).strip(),
            description=str(entry.description).strip(),
            category_slugs=_normalized_category_slugs(entry.category_slugs),
        )
        updated_entries: list[SupercategoryEntry] = []
        created = True
        for existing in manifest.supercategories:
            if existing.slug == normalized_entry.slug:
                updated_entries.append(normalized_entry)
                created = False
                continue
            updated_entries.append(existing)
        if created:
            updated_entries.append(normalized_entry)
        self.save(
            SupercategoryManifest(
                schema_version=manifest.schema_version,
                supercategories=updated_entries,
            )
        )
        return normalized_entry, created

    def assign_category(self, *, category_slug: str, supercategory_slug: str) -> str | None:
        category_slug = validate_category_slug(category_slug)
        supercategory_slug = validate_supercategory_slug(supercategory_slug)
        manifest = self.load_manifest_or_default()
        updated_entries: list[SupercategoryEntry] = []
        previous_supercategory_slug: str | None = None
        found_target = False
        for entry in manifest.supercategories:
            category_slugs = [
                slug
                for slug in _normalized_category_slugs(entry.category_slugs)
                if slug != category_slug
            ]
            if category_slug in entry.category_slugs and previous_supercategory_slug is None:
                previous_supercategory_slug = entry.slug
            if entry.slug == supercategory_slug:
                found_target = True
                if category_slug not in category_slugs:
                    category_slugs.append(category_slug)
            updated_entries.append(
                SupercategoryEntry(
                    slug=entry.slug,
                    title=entry.title,
                    description=entry.description,
                    category_slugs=category_slugs,
                )
            )
        if not found_target:
            raise ValueError(f"Supercategory not found: {supercategory_slug}")
        self.save(
            SupercategoryManifest(
                schema_version=manifest.schema_version,
                supercategories=updated_entries,
            )
        )
        return previous_supercategory_slug

    def remove_category(self, category_slug: str) -> str | None:
        category_slug = validate_category_slug(category_slug)
        manifest = self.load_manifest()
        if manifest is None:
            return None
        updated_entries: list[SupercategoryEntry] = []
        previous_supercategory_slug: str | None = None
        for entry in manifest.supercategories:
            category_slugs = [
                slug
                for slug in _normalized_category_slugs(entry.category_slugs)
                if slug != category_slug
            ]
            if category_slug in entry.category_slugs and previous_supercategory_slug is None:
                previous_supercategory_slug = entry.slug
            updated_entries.append(
                SupercategoryEntry(
                    slug=entry.slug,
                    title=entry.title,
                    description=entry.description,
                    category_slugs=category_slugs,
                )
            )
        self.save(
            SupercategoryManifest(
                schema_version=manifest.schema_version,
                supercategories=updated_entries,
            )
        )
        return previous_supercategory_slug

    def delete_supercategory(self, supercategory_slug: str) -> SupercategoryEntry | None:
        supercategory_slug = validate_supercategory_slug(supercategory_slug)
        manifest = self.load_manifest()
        if manifest is None:
            return None
        deleted_entry: SupercategoryEntry | None = None
        updated_entries: list[SupercategoryEntry] = []
        for entry in manifest.supercategories:
            if entry.slug == supercategory_slug and deleted_entry is None:
                deleted_entry = entry
                continue
            updated_entries.append(entry)
        if deleted_entry is None:
            return None
        self.save(
            SupercategoryManifest(
                schema_version=manifest.schema_version,
                supercategories=updated_entries,
            )
        )
        return deleted_entry

    def build_category_to_supercategory_map(self) -> dict[str, str]:
        manifest = self.load_manifest()
        if manifest is None:
            return {}
        result: dict[str, str] = {}
        for entry in manifest.supercategories:
            for slug in entry.category_slugs:
                if slug not in result:
                    result[slug] = entry.slug
        return result
