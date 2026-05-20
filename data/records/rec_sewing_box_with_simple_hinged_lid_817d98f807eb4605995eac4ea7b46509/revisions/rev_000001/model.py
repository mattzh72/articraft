from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="sewing_box")

    body_paint = model.material("body_paint", rgba=(0.70, 0.56, 0.61, 1.0))
    lid_paint = model.material("lid_paint", rgba=(0.78, 0.65, 0.69, 1.0))
    interior = model.material("interior", rgba=(0.87, 0.80, 0.69, 1.0))

    depth = 0.24
    width = 0.16
    height = 0.09
    wall = 0.007
    bottom = 0.006
    lid_thickness = 0.008
    lid_overhang = 0.003

    body = model.part("body")
    body.visual(
        Box((depth, width, bottom)),
        origin=Origin(xyz=(0.0, 0.0, bottom / 2.0)),
        material=interior,
        name="bottom_panel",
    )
    body.visual(
        Box((wall, width, height)),
        origin=Origin(xyz=(-depth / 2.0 + wall / 2.0, 0.0, height / 2.0)),
        material=body_paint,
        name="rear_wall",
    )
    body.visual(
        Box((wall, width, height)),
        origin=Origin(xyz=(depth / 2.0 - wall / 2.0, 0.0, height / 2.0)),
        material=body_paint,
        name="front_wall",
    )
    body.visual(
        Box((depth - 2.0 * wall, wall, height)),
        origin=Origin(xyz=(0.0, -width / 2.0 + wall / 2.0, height / 2.0)),
        material=body_paint,
        name="left_wall",
    )
    body.visual(
        Box((depth - 2.0 * wall, wall, height)),
        origin=Origin(xyz=(0.0, width / 2.0 - wall / 2.0, height / 2.0)),
        material=body_paint,
        name="right_wall",
    )
    body.inertial = Inertial.from_geometry(
        Box((depth, width, height)),
        mass=0.85,
        origin=Origin(xyz=(0.0, 0.0, height / 2.0)),
    )

    lid = model.part("lid")
    lid.visual(
        Box((depth + lid_overhang, width + 2.0 * lid_overhang, lid_thickness)),
        origin=Origin(
            xyz=((depth + lid_overhang) / 2.0, 0.0, lid_thickness / 2.0)
        ),
        material=lid_paint,
        name="lid_panel",
    )
    lid.inertial = Inertial.from_geometry(
        Box((depth + lid_overhang, width + 2.0 * lid_overhang, lid_thickness)),
        mass=0.22,
        origin=Origin(
            xyz=((depth + lid_overhang) / 2.0, 0.0, lid_thickness / 2.0)
        ),
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(-depth / 2.0, 0.0, height)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=3.0,
            lower=0.0,
            upper=1.9,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    hinge = object_model.get_articulation("body_to_lid")

    ctx.expect_gap(
        lid,
        body,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        name="closed lid sits on the box rim",
    )
    ctx.expect_overlap(
        lid,
        body,
        axes="xy",
        min_overlap=0.14,
        name="lid covers the shallow box body",
    )

    closed_aabb = ctx.part_element_world_aabb(lid, elem="lid_panel")
    with ctx.pose({hinge: 1.2}):
        opened_aabb = ctx.part_element_world_aabb(lid, elem="lid_panel")

    ctx.check(
        "lid opens upward from the rear hinge",
        closed_aabb is not None
        and opened_aabb is not None
        and opened_aabb[1][2] > closed_aabb[1][2] + 0.10,
        details=f"closed_aabb={closed_aabb}, opened_aabb={opened_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
