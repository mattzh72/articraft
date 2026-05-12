from __future__ import annotations

from viewer.api.schemas import CategoryOptionResponse, SupercategoryOptionResponse


class ViewerStoreTaxonomyMixin:
    def list_supercategories(self) -> list[SupercategoryOptionResponse]:
        manifest = self.supercategory_store.load_manifest()
        if manifest is None:
            return []
        return [
            SupercategoryOptionResponse(
                slug=entry.slug,
                title=entry.title,
                description=entry.description,
                category_slugs=list(entry.category_slugs),
            )
            for entry in manifest.supercategories
        ]

    def list_categories(self) -> list[CategoryOptionResponse]:
        categories_by_slug: dict[str, str] = {}

        for category_path in self.repo.layout.categories_root.glob("*/category.json"):
            category = self.repo.read_json(category_path)
            if not isinstance(category, dict):
                continue
            slug = str(category.get("slug") or "").strip()
            if not slug:
                continue
            title = (
                str(category.get("title") or "").strip()
                or slug.replace("_", " ").replace("-", " ").title()
            )
            categories_by_slug[slug] = title

        records_root = self.repo.layout.records_root
        if records_root.exists():
            for record_dir in records_root.iterdir():
                if not record_dir.is_dir():
                    continue
                record = self.repo.read_json(record_dir / "record.json")
                if not isinstance(record, dict):
                    continue
                slug = str(record.get("category_slug") or "").strip()
                if not slug or slug in categories_by_slug:
                    continue
                categories_by_slug[slug] = slug.replace("_", " ").replace("-", " ").title()

        cat_to_super = self.supercategory_store.build_category_to_supercategory_map()

        return [
            CategoryOptionResponse(
                slug=slug,
                title=title,
                supercategory_slug=cat_to_super.get(slug),
            )
            for slug, title in sorted(
                categories_by_slug.items(),
                key=lambda item: (item[1].lower(), item[0]),
            )
        ]
