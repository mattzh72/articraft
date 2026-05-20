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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def _polar(radius: float, angle: float) -> tuple[float, float]:
    return (radius * math.cos(angle), radius * math.sin(angle))


def _add_chord_panel(
    part,
    *,
    radius: float,
    center_angle: float,
    span_angle: float,
    height: float,
    thickness: float,
    z_center: float,
    material,
    name: str,
) -> None:
    chord_length = 2.0 * radius * math.sin(span_angle * 0.5)
    mid_radius = radius * math.cos(span_angle * 0.5)
    px, py = _polar(mid_radius, center_angle)
    part.visual(
        Box((chord_length, thickness, height)),
        origin=Origin(xyz=(px, py, z_center), rpy=(0.0, 0.0, center_angle + math.pi / 2.0)),
        material=material,
        name=name,
    )


def _build_wing(model: ArticulatedObject, name: str, metal, glass):
    wing = model.part(name=name)

    wing.visual(
        Box((0.06, 0.055, 2.00)),
        origin=Origin(xyz=(0.08, 0.0, 1.00)),
        material=metal,
        name="inner_stile",
    )
    wing.visual(
        Box((0.70, 0.05, 0.08)),
        origin=Origin(xyz=(0.40, 0.0, 0.04)),
        material=metal,
        name="bottom_rail",
    )
    wing.visual(
        Box((0.70, 0.05, 0.08)),
        origin=Origin(xyz=(0.40, 0.0, 1.96)),
        material=metal,
        name="top_rail",
    )
    wing.visual(
        Box((0.05, 0.06, 2.00)),
        origin=Origin(xyz=(0.765, 0.0, 1.00)),
        material=metal,
        name="outer_stile",
    )
    wing.visual(
        Box((0.63, 0.016, 1.84)),
        origin=Origin(xyz=(0.425, 0.0, 1.00)),
        material=glass,
        name="glass_leaf",
    )
    wing.inertial = Inertial.from_geometry(
        Box((0.80, 0.06, 2.00)),
        mass=28.0,
        origin=Origin(xyz=(0.40, 0.0, 1.00)),
    )
    return wing


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="corner_three_wing_revolving_door")

    frame_metal = model.material("frame_metal", rgba=(0.42, 0.44, 0.47, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.67, 0.69, 0.71, 1.0))
    glass = model.material("glass", rgba=(0.70, 0.83, 0.89, 0.28))
    wall_stone = model.material("wall_stone", rgba=(0.82, 0.82, 0.79, 1.0))
    floor_stone = model.material("floor_stone", rgba=(0.54, 0.56, 0.58, 1.0))

    frame = model.part("corner_frame")
    frame.visual(
        Box((2.40, 2.40, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=floor_stone,
        name="floor_slab",
    )
    frame.visual(
        Cylinder(radius=1.02, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, 2.27)),
        material=frame_metal,
        name="canopy",
    )
    frame.visual(
        Box((0.14, 1.28, 2.16)),
        origin=Origin(xyz=(-0.91, -0.28, 1.08)),
        material=wall_stone,
        name="corner_wall_x",
    )
    frame.visual(
        Box((1.28, 0.14, 2.16)),
        origin=Origin(xyz=(-0.28, -0.91, 1.08)),
        material=wall_stone,
        name="corner_wall_y",
    )
    frame.visual(
        Box((0.18, 0.18, 2.16)),
        origin=Origin(xyz=(-0.91, -0.91, 1.08)),
        material=frame_metal,
        name="corner_pier",
    )
    frame.visual(
        Box((0.44, 0.022, 2.00)),
        origin=Origin(xyz=(-0.63, 0.40, 1.05)),
        material=glass,
        name="return_glass_x",
    )
    frame.visual(
        Box((0.022, 0.44, 2.00)),
        origin=Origin(xyz=(0.40, -0.63, 1.05)),
        material=glass,
        name="return_glass_y",
    )

    drum_post_radius = 0.98
    for idx, deg in enumerate((0.0, 60.0, 120.0, 180.0, 240.0, 300.0)):
        angle = math.radians(deg)
        px, py = _polar(drum_post_radius, angle)
        frame.visual(
            Cylinder(radius=0.025, length=2.16),
            origin=Origin(xyz=(px, py, 1.13)),
            material=frame_metal,
            name=f"drum_post_{idx}",
        )

    fixed_panel_radius = 0.93
    fixed_panel_span = math.radians(54.0)
    for idx, deg in enumerate((60.0, 180.0, 300.0)):
        _add_chord_panel(
            frame,
            radius=fixed_panel_radius,
            center_angle=math.radians(deg),
            span_angle=fixed_panel_span,
            height=2.00,
            thickness=0.02,
            z_center=1.05,
            material=glass,
            name=f"fixed_glass_{idx}",
        )

    frame.inertial = Inertial.from_geometry(
        Box((2.40, 2.40, 2.39)),
        mass=240.0,
        origin=Origin(xyz=(0.0, 0.0, 1.195)),
    )

    rotor = model.part("rotor")
    rotor.visual(
        Cylinder(radius=0.05, length=2.03),
        origin=Origin(xyz=(0.0, 0.0, 1.015)),
        material=brushed_steel,
        name="center_post",
    )
    rotor.visual(
        Cylinder(radius=0.048, length=0.04),
        origin=Origin(xyz=(0.0, 0.0, 0.02)),
        material=frame_metal,
        name="bottom_hub",
    )
    rotor.visual(
        Cylinder(radius=0.048, length=0.04),
        origin=Origin(xyz=(0.0, 0.0, 2.01)),
        material=frame_metal,
        name="top_hub",
    )
    rotor.visual(
        Cylinder(radius=0.048, length=0.14),
        origin=Origin(xyz=(0.0, 0.0, 1.015)),
        material=frame_metal,
        name="mid_collar",
    )
    rotor.inertial = Inertial.from_geometry(
        Cylinder(radius=0.05, length=2.03),
        mass=36.0,
        origin=Origin(xyz=(0.0, 0.0, 1.015)),
    )

    wing_a = _build_wing(model, "wing_a", frame_metal, glass)
    wing_b = _build_wing(model, "wing_b", frame_metal, glass)
    wing_c = _build_wing(model, "wing_c", frame_metal, glass)

    spin_joint = model.articulation(
        "frame_to_rotor",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=rotor,
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=60.0, velocity=2.5),
    )
    model.articulation(
        "rotor_to_wing_a",
        ArticulationType.FIXED,
        parent=rotor,
        child=wing_a,
        origin=Origin(),
    )
    model.articulation(
        "rotor_to_wing_b",
        ArticulationType.FIXED,
        parent=rotor,
        child=wing_b,
        origin=Origin(rpy=(0.0, 0.0, 2.0 * math.pi / 3.0)),
    )
    model.articulation(
        "rotor_to_wing_c",
        ArticulationType.FIXED,
        parent=rotor,
        child=wing_c,
        origin=Origin(rpy=(0.0, 0.0, 4.0 * math.pi / 3.0)),
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

    frame = object_model.get_part("corner_frame")
    rotor = object_model.get_part("rotor")
    wing_a = object_model.get_part("wing_a")
    wing_b = object_model.get_part("wing_b")
    wing_c = object_model.get_part("wing_c")
    spin_joint = object_model.get_articulation("frame_to_rotor")

    ctx.expect_contact(
        wing_a,
        rotor,
        elem_a="inner_stile",
        elem_b="center_post",
        contact_tol=0.002,
        name="wing_a is mounted to the central post",
    )
    ctx.expect_contact(
        wing_b,
        rotor,
        elem_a="inner_stile",
        elem_b="center_post",
        contact_tol=0.002,
        name="wing_b is mounted to the central post",
    )
    ctx.expect_contact(
        wing_c,
        rotor,
        elem_a="inner_stile",
        elem_b="center_post",
        contact_tol=0.002,
        name="wing_c is mounted to the central post",
    )

    for wing_name, wing in (("wing_a", wing_a), ("wing_b", wing_b), ("wing_c", wing_c)):
        ctx.expect_within(
            wing,
            frame,
            axes="xy",
            inner_elem="outer_stile",
            outer_elem="canopy",
            margin=0.02,
            name=f"{wing_name} stays within the revolving drum footprint",
        )

    def aabb_center(aabb):
        if aabb is None:
            return None
        (min_x, min_y, min_z), (max_x, max_y, max_z) = aabb
        return (
            0.5 * (min_x + max_x),
            0.5 * (min_y + max_y),
            0.5 * (min_z + max_z),
        )

    outer_centers = []
    for wing in (wing_a, wing_b, wing_c):
        outer_aabb = ctx.part_element_world_aabb(wing, elem="outer_stile")
        outer_centers.append(aabb_center(outer_aabb))

    angle_values = []
    for center in outer_centers:
        if center is not None:
            angle = math.atan2(center[1], center[0])
            if angle < 0.0:
                angle += 2.0 * math.pi
            angle_values.append(angle)
    angle_values.sort()
    angle_gaps = []
    if len(angle_values) == 3:
        for idx in range(3):
            next_idx = (idx + 1) % 3
            raw_gap = angle_values[next_idx] - angle_values[idx]
            if next_idx == 0:
                raw_gap += 2.0 * math.pi
            angle_gaps.append(raw_gap)

    ctx.check(
        "three wings are evenly spaced around the post",
        len(angle_gaps) == 3 and all(abs(gap - 2.0 * math.pi / 3.0) < 0.08 for gap in angle_gaps),
        details=f"angles={angle_values}, gaps={angle_gaps}",
    )

    rest_outer = aabb_center(ctx.part_element_world_aabb(wing_a, elem="outer_stile"))
    with ctx.pose({spin_joint: math.radians(30.0)}):
        turned_outer = aabb_center(ctx.part_element_world_aabb(wing_a, elem="outer_stile"))

    rest_radius = None
    turned_radius = None
    if rest_outer is not None:
        rest_radius = math.hypot(rest_outer[0], rest_outer[1])
    if turned_outer is not None:
        turned_radius = math.hypot(turned_outer[0], turned_outer[1])

    ctx.check(
        "wing assembly rotates continuously about the vertical post axis",
        rest_outer is not None
        and turned_outer is not None
        and turned_outer[1] > rest_outer[1] + 0.20
        and turned_outer[0] < rest_outer[0] - 0.08
        and rest_radius is not None
        and turned_radius is not None
        and abs(rest_radius - turned_radius) < 0.03,
        details=(
            f"rest_outer={rest_outer}, turned_outer={turned_outer}, "
            f"rest_radius={rest_radius}, turned_radius={turned_radius}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
