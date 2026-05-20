from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    tube_from_spline_points,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="weatherproof_padlock")

    rubber = model.material("rubber_charcoal", rgba=(0.14, 0.15, 0.16, 1.0))
    guide_dark = model.material("guide_dark", rgba=(0.08, 0.08, 0.09, 1.0))
    cover_rubber = model.material("cover_rubber", rgba=(0.18, 0.19, 0.20, 1.0))
    steel = model.material("steel", rgba=(0.77, 0.79, 0.81, 1.0))
    brass = model.material("brass", rgba=(0.72, 0.58, 0.24, 1.0))

    body_width = 0.050
    body_depth = 0.030
    body_height = 0.056

    shackle_radius = 0.0042
    leg_spacing = 0.038
    shackle_peak = 0.080

    body = model.part("body")
    shell_mesh = _mesh(
        "padlock_body_shell",
        ExtrudeGeometry.from_z0(
            rounded_rect_profile(body_depth, body_width, 0.0055),
            body_height,
            cap=True,
            closed=True,
        ),
    )
    body.visual(shell_mesh, material=rubber, name="body_shell")

    guide_x = body_depth / 2.0 + 0.0009
    guide_y_center = 0.007
    guide_span_y = 0.035
    guide_thickness = 0.0022
    guide_rail_height = 0.0020
    cover_z = 0.018

    body.visual(
        Box((guide_thickness, guide_span_y, guide_rail_height)),
        origin=Origin(xyz=(guide_x, guide_y_center, cover_z + 0.0067)),
        material=guide_dark,
        name="guide_rail_top",
    )
    body.visual(
        Box((guide_thickness, guide_span_y, guide_rail_height)),
        origin=Origin(xyz=(guide_x, guide_y_center, cover_z - 0.0067)),
        material=guide_dark,
        name="guide_rail_bottom",
    )
    body.visual(
        Box((guide_thickness, 0.003, 0.016)),
        origin=Origin(xyz=(guide_x, -0.0105, cover_z)),
        material=guide_dark,
        name="guide_stop",
    )
    body.visual(
        Cylinder(radius=0.0040, length=0.0006),
        origin=Origin(
            xyz=(body_depth / 2.0 + 0.0002, 0.0, cover_z),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=brass,
        name="keyway_bezel",
    )
    body.inertial = Inertial.from_geometry(
        Box((body_depth, body_width, body_height)),
        mass=0.65,
        origin=Origin(xyz=(0.0, 0.0, body_height / 2.0)),
    )

    shackle = model.part("shackle")
    shackle_path = [
        (0.0, 0.0, 0.0),
        (0.0, 0.0, 0.028),
        (0.0, 0.004, 0.058),
        (0.0, 0.011, 0.073),
        (0.0, 0.019, shackle_peak),
        (0.0, 0.027, 0.073),
        (0.0, 0.034, 0.058),
        (0.0, leg_spacing, 0.028),
        (0.0, leg_spacing, 0.0),
    ]
    shackle.visual(
        _mesh(
            "padlock_shackle",
            tube_from_spline_points(
                shackle_path,
                radius=shackle_radius,
                samples_per_segment=18,
                radial_segments=20,
                cap_ends=True,
            ),
        ),
        material=steel,
        name="shackle_rod",
    )
    shackle.inertial = Inertial.from_geometry(
        Box((shackle_radius * 2.5, leg_spacing + 0.004, shackle_peak + shackle_radius)),
        mass=0.22,
        origin=Origin(
            xyz=(
                0.0,
                leg_spacing / 2.0,
                (shackle_peak + shackle_radius) / 2.0,
            )
        ),
    )

    key_cover = model.part("key_cover")
    cover_thickness = 0.0024
    cover_width = 0.017
    cover_height = 0.0114
    key_cover.visual(
        Box((cover_thickness, cover_width, cover_height)),
        material=cover_rubber,
        name="cover_plate",
    )
    key_cover.visual(
        Box((0.0030, 0.0032, 0.0060)),
        origin=Origin(xyz=(0.0004, 0.0065, 0.0)),
        material=guide_dark,
        name="cover_tab",
    )
    key_cover.inertial = Inertial.from_geometry(
        Box((0.0030, 0.020, 0.012)),
        mass=0.03,
    )

    model.articulation(
        "body_to_shackle",
        ArticulationType.REVOLUTE,
        parent=body,
        child=shackle,
        origin=Origin(xyz=(0.0, -leg_spacing / 2.0, body_height)),
        # The closed shackle lies mainly along +Y from the retained leg.
        # -Z makes positive q swing the free leg toward +X, i.e. outward/front.
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=2.0,
            lower=0.0,
            upper=math.radians(125.0),
        ),
    )
    model.articulation(
        "body_to_key_cover",
        ArticulationType.PRISMATIC,
        parent=body,
        child=key_cover,
        origin=Origin(
            xyz=(body_depth / 2.0 + 0.0009 + cover_thickness / 2.0, 0.0, cover_z)
        ),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=0.05,
            lower=0.0,
            upper=0.014,
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
    shackle = object_model.get_part("shackle")
    key_cover = object_model.get_part("key_cover")
    shackle_joint = object_model.get_articulation("body_to_shackle")
    cover_joint = object_model.get_articulation("body_to_key_cover")

    ctx.expect_gap(
        shackle,
        body,
        axis="z",
        max_gap=0.001,
        max_penetration=0.00005,
        name="shackle rises flush from the top of the body",
    )
    ctx.expect_gap(
        key_cover,
        body,
        axis="x",
        min_gap=0.0006,
        max_gap=0.0014,
        negative_elem="body_shell",
        name="key cover rides just proud of the front shell",
    )
    ctx.expect_overlap(
        key_cover,
        body,
        axes="z",
        min_overlap=0.009,
        name="key cover stays aligned with the front guide height",
    )

    closed_cover_pos = ctx.part_world_position(key_cover)
    with ctx.pose({cover_joint: 0.014}):
        open_cover_pos = ctx.part_world_position(key_cover)
        ctx.expect_gap(
            key_cover,
            body,
            axis="x",
            min_gap=0.0006,
            max_gap=0.0014,
            negative_elem="body_shell",
            name="open key cover remains in front of the shell",
        )
    ctx.check(
        "key cover slides laterally across the front guide",
        closed_cover_pos is not None
        and open_cover_pos is not None
        and open_cover_pos[1] > closed_cover_pos[1] + 0.010,
        details=f"closed={closed_cover_pos}, open={open_cover_pos}",
    )

    closed_shackle_aabb = ctx.part_world_aabb(shackle)
    with ctx.pose({shackle_joint: math.radians(110.0)}):
        open_shackle_aabb = ctx.part_world_aabb(shackle)
    ctx.check(
        "shackle swings outward on its retained leg",
        closed_shackle_aabb is not None
        and open_shackle_aabb is not None
        and open_shackle_aabb[1][0] > closed_shackle_aabb[1][0] + 0.020,
        details=f"closed={closed_shackle_aabb}, open={open_shackle_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
