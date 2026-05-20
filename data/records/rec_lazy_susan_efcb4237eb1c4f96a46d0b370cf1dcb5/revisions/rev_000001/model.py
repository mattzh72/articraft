from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


POST_RADIUS = 0.019
COLLAR_CLEARANCE_RADIUS = 0.021
COLLAR_OUTER_RADIUS = 0.042


def _annular_mesh(name: str, *, inner_radius: float, outer_radius: float, z0: float, z1: float):
    profile = [
        (inner_radius, z0),
        (outer_radius, z0),
        (outer_radius, z1),
        (inner_radius, z1),
    ]
    return mesh_from_geometry(LatheGeometry(profile, segments=96), name)


def _add_bearing_pads(part, *, height: float, radius: float, material) -> None:
    pad_center_radius = POST_RADIUS + radius
    for index, angle in enumerate((0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0)):
        x = pad_center_radius * math.cos(angle)
        y = pad_center_radius * math.sin(angle)
        part.visual(
            Cylinder(radius=radius, length=height),
            origin=Origin(xyz=(x, y, 0.0)),
            material=material,
            name="bearing_pad_front" if index == 0 else f"bearing_pad_{index}",
        )


def _build_tier(
    model: ArticulatedObject,
    *,
    name: str,
    shelf_radius: float,
    board_thickness: float,
    lip_height: float,
    collar_height: float,
    board_material,
    collar_material,
    bearing_material,
):
    tier = model.part(name)

    collar_shell = _annular_mesh(
        f"{name}_collar_shell",
        inner_radius=COLLAR_CLEARANCE_RADIUS,
        outer_radius=COLLAR_OUTER_RADIUS,
        z0=-collar_height / 2.0,
        z1=collar_height / 2.0,
    )
    board_ring = _annular_mesh(
        f"{name}_board_ring",
        inner_radius=COLLAR_OUTER_RADIUS,
        outer_radius=shelf_radius,
        z0=collar_height / 2.0,
        z1=collar_height / 2.0 + board_thickness,
    )
    outer_lip = _annular_mesh(
        f"{name}_outer_lip",
        inner_radius=shelf_radius - 0.018,
        outer_radius=shelf_radius,
        z0=collar_height / 2.0 + board_thickness,
        z1=collar_height / 2.0 + board_thickness + lip_height,
    )

    tier.visual(collar_shell, material=collar_material, name="bearing_collar")
    _add_bearing_pads(
        tier,
        height=collar_height,
        radius=0.005,
        material=bearing_material,
    )
    tier.visual(board_ring, material=board_material, name="shelf_board")
    tier.visual(outer_lip, material=board_material, name="shelf_lip")

    overall_height = collar_height + board_thickness + lip_height
    tier.inertial = Inertial.from_geometry(
        Cylinder(radius=shelf_radius, length=overall_height),
        mass=1.15 if shelf_radius > 0.15 else 0.85,
        origin=Origin(xyz=(0.0, 0.0, -collar_height / 2.0 + overall_height / 2.0)),
    )
    return tier


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="two_tier_lazy_susan")

    base_black = model.material("base_black", rgba=(0.14, 0.14, 0.15, 1.0))
    oak = model.material("oak", rgba=(0.74, 0.60, 0.42, 1.0))
    brass = model.material("brass", rgba=(0.74, 0.63, 0.34, 1.0))
    bearing_steel = model.material("bearing_steel", rgba=(0.58, 0.60, 0.63, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.08, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.115, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.017)),
        material=base_black,
        name="base_disc",
    )
    for index, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        x = 0.072 * math.cos(angle)
        y = 0.072 * math.sin(angle)
        base.visual(
            Cylinder(radius=0.012, length=0.006),
            origin=Origin(xyz=(x, y, 0.003)),
            material=rubber,
            name=f"foot_{index}",
        )
    base.inertial = Inertial.from_geometry(
        Cylinder(radius=0.115, length=0.028),
        mass=2.8,
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
    )

    post = model.part("central_post")
    post.visual(
        Cylinder(radius=POST_RADIUS, length=0.420),
        origin=Origin(xyz=(0.0, 0.0, 0.210)),
        material=base_black,
        name="post_shaft",
    )
    post.visual(
        Cylinder(radius=0.028, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.428)),
        material=brass,
        name="post_cap",
    )
    post.inertial = Inertial.from_geometry(
        Cylinder(radius=0.028, length=0.436),
        mass=1.2,
        origin=Origin(xyz=(0.0, 0.0, 0.218)),
    )

    lower_tier = _build_tier(
        model,
        name="lower_tier",
        shelf_radius=0.175,
        board_thickness=0.014,
        lip_height=0.022,
        collar_height=0.028,
        board_material=oak,
        collar_material=brass,
        bearing_material=bearing_steel,
    )
    upper_tier = _build_tier(
        model,
        name="upper_tier",
        shelf_radius=0.135,
        board_thickness=0.012,
        lip_height=0.020,
        collar_height=0.028,
        board_material=oak,
        collar_material=brass,
        bearing_material=bearing_steel,
    )

    model.articulation(
        "base_to_post",
        ArticulationType.FIXED,
        parent=base,
        child=post,
        origin=Origin(xyz=(0.0, 0.0, 0.028)),
    )
    model.articulation(
        "post_to_lower_tier",
        ArticulationType.REVOLUTE,
        parent=post,
        child=lower_tier,
        origin=Origin(xyz=(0.0, 0.0, 0.095)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=5.0,
            velocity=3.0,
            lower=-math.pi,
            upper=math.pi,
        ),
    )
    model.articulation(
        "post_to_upper_tier",
        ArticulationType.REVOLUTE,
        parent=post,
        child=upper_tier,
        origin=Origin(xyz=(0.0, 0.0, 0.275)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=5.0,
            velocity=3.0,
            lower=-math.pi,
            upper=math.pi,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    post = object_model.get_part("central_post")
    lower_tier = object_model.get_part("lower_tier")
    upper_tier = object_model.get_part("upper_tier")
    lower_spin = object_model.get_articulation("post_to_lower_tier")
    upper_spin = object_model.get_articulation("post_to_upper_tier")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "lower_tier_axis_vertical",
        tuple(lower_spin.axis) == (0.0, 0.0, 1.0),
        f"expected (0, 0, 1), got {lower_spin.axis}",
    )
    ctx.check(
        "upper_tier_axis_vertical",
        tuple(upper_spin.axis) == (0.0, 0.0, 1.0),
        f"expected (0, 0, 1), got {upper_spin.axis}",
    )

    ctx.expect_contact(base, post, elem_a="base_disc", elem_b="post_shaft")
    ctx.expect_contact(lower_tier, post, elem_a="bearing_pad_front", elem_b="post_shaft")
    ctx.expect_contact(upper_tier, post, elem_a="bearing_pad_front", elem_b="post_shaft")

    ctx.expect_gap(
        lower_tier,
        base,
        axis="z",
        min_gap=0.080,
        positive_elem="shelf_board",
        negative_elem="base_disc",
    )
    ctx.expect_gap(
        upper_tier,
        lower_tier,
        axis="z",
        min_gap=0.100,
        positive_elem="shelf_board",
        negative_elem="shelf_lip",
    )
    ctx.expect_within(
        upper_tier,
        lower_tier,
        axes="xy",
        margin=0.0,
        inner_elem="shelf_board",
        outer_elem="shelf_board",
    )

    with ctx.pose({lower_spin: 0.85, upper_spin: -0.60}):
        ctx.expect_contact(lower_tier, post, elem_a="bearing_pad_front", elem_b="post_shaft")
        ctx.expect_contact(upper_tier, post, elem_a="bearing_pad_front", elem_b="post_shaft")
        ctx.expect_gap(
            upper_tier,
            lower_tier,
            axis="z",
            min_gap=0.100,
            positive_elem="shelf_board",
            negative_elem="shelf_lip",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
