from __future__ import annotations

from math import cos, pi, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    superellipse_profile,
)


def _translated_profile(profile: list[tuple[float, float]], dx: float, dz: float) -> list[tuple[float, float]]:
    return [(x + dx, z + dz) for x, z in profile]


def _circle_profile(radius: float, *, cx: float = 0.0, cz: float = 0.0, segments: int = 36) -> list[tuple[float, float]]:
    return [
        (cx + radius * cos(2.0 * pi * i / segments), cz + radius * sin(2.0 * pi * i / segments))
        for i in range(segments)
    ]


def _capsule_profile(length: float, width: float, *, cap_segments: int = 18) -> list[tuple[float, float]]:
    radius = width * 0.5
    points: list[tuple[float, float]] = []
    for i in range(cap_segments + 1):
        a = -pi / 2.0 + pi * i / cap_segments
        points.append((length + radius * cos(a), radius * sin(a)))
    for i in range(cap_segments + 1):
        a = pi / 2.0 + pi * i / cap_segments
        points.append((radius * cos(a), radius * sin(a)))
    return points


def _flat_link_mesh(name: str, *, length: float, width: float, thickness: float, hole_diameter: float):
    outer = _capsule_profile(length, width)
    hole_r = hole_diameter * 0.5
    holes = [
        _circle_profile(hole_r, cx=0.0, cz=0.0, segments=32),
        _circle_profile(hole_r, cx=length, cz=0.0, segments=32),
    ]
    return mesh_from_geometry(
        ExtrudeWithHolesGeometry(outer, holes, height=thickness, center=True).rotate_x(pi / 2.0),
        name,
    )


def _pad_mesh(name: str, *, link_length: float, pad_length: float, pad_height: float, thickness: float):
    pad_center_x = link_length + pad_length * 0.5 - 0.012
    outer = _translated_profile(
        rounded_rect_profile(pad_length, pad_height, radius=0.010, corner_segments=6),
        pad_center_x,
        0.0,
    )
    holes = [
        _circle_profile(0.004, cx=pad_center_x - pad_length * 0.22, cz=pad_height * 0.24, segments=28),
        _circle_profile(0.004, cx=pad_center_x + pad_length * 0.22, cz=-pad_height * 0.24, segments=28),
    ]
    return mesh_from_geometry(
        ExtrudeWithHolesGeometry(outer, holes, height=thickness, center=True).rotate_x(pi / 2.0),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="three_bar_folding_stay")

    dark_steel = model.material("blackened_steel", rgba=(0.08, 0.085, 0.09, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.62, 0.64, 0.66, 1.0))
    zinc = model.material("zinc_pin_heads", rgba=(0.78, 0.77, 0.72, 1.0))
    screw_dark = model.material("dark_screw_heads", rgba=(0.025, 0.025, 0.028, 1.0))

    link_width = 0.032
    link_thickness = 0.006
    hole_diameter = 0.010
    cap_radius = 0.0085
    cap_thickness = 0.003
    link_1_len = 0.220
    link_2_len = 0.200
    link_3_len = 0.180

    base_bracket = model.part("base_bracket")
    # Wall plate and fork cheeks are built from overlapping plate members so the
    # root part reads as one bracket with a clear central gap for the first bar.
    base_bracket.visual(
        Box((0.010, 0.100, 0.165)),
        origin=Origin(xyz=(-0.050, 0.0, 0.0)),
        material=dark_steel,
        name="mounting_plate",
    )
    for y in (-0.013, 0.013):
        base_bracket.visual(
            Box((0.026, 0.008, 0.044)),
            origin=Origin(xyz=(-0.034, y, 0.0)),
            material=dark_steel,
            name=f"side_standoff_{'n' if y < 0 else 'p'}",
        )
        base_bracket.visual(
            Box((0.056, 0.006, 0.052)),
            origin=Origin(xyz=(0.005, y, 0.0)),
            material=dark_steel,
            name=f"fork_cheek_{'n' if y < 0 else 'p'}",
        )
        base_bracket.visual(
            Cylinder(radius=0.016, length=0.0035),
            origin=Origin(xyz=(0.0, 0.0174 if y > 0 else -0.0174, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
            material=zinc,
            name=f"outer_pivot_washer_{'n' if y < 0 else 'p'}",
        )

    base_bracket.visual(
        Cylinder(radius=0.0052, length=0.036),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=zinc,
        name="base_pivot_pin",
    )

    for sx in (-1.0, 1.0):
        for sz in (-1.0, 1.0):
            base_bracket.visual(
                Cylinder(radius=0.0065, length=0.004),
                origin=Origin(xyz=(-0.056, sx * 0.030, sz * 0.055), rpy=(0.0, pi / 2.0, 0.0)),
                material=screw_dark,
                name=f"plate_screw_{'n' if sx < 0 else 'p'}_{'l' if sz < 0 else 'u'}",
            )

    link_1 = model.part("link_1")
    link_1.visual(
        _flat_link_mesh("link_1_bar", length=link_1_len, width=link_width, thickness=link_thickness, hole_diameter=hole_diameter),
        origin=Origin(),
        material=brushed_steel,
        name="slotted_bar",
    )
    for x, label in ((0.0, "root"), (link_1_len, "tip")):
        link_1.visual(
            Cylinder(radius=cap_radius, length=cap_thickness),
            origin=Origin(xyz=(x, -0.0044, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
            material=zinc,
            name=f"{label}_rivet_cap",
        )
    link_1.visual(
        Cylinder(radius=0.0052, length=0.030),
        origin=Origin(xyz=(link_1_len, 0.006, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=zinc,
        name="tip_pivot_pin",
    )

    link_2 = model.part("link_2")
    link_2.visual(
        _flat_link_mesh("link_2_bar", length=link_2_len, width=link_width, thickness=link_thickness, hole_diameter=hole_diameter),
        origin=Origin(xyz=(0.0, 0.012, 0.0)),
        material=brushed_steel,
        name="slotted_bar",
    )
    for x, label in ((0.0, "root"), (link_2_len, "tip")):
        link_2.visual(
            Cylinder(radius=cap_radius, length=cap_thickness),
            origin=Origin(xyz=(x, 0.0164, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
            material=zinc,
            name=f"{label}_rivet_cap",
        )
    link_2.visual(
        Cylinder(radius=0.0052, length=0.040),
        origin=Origin(xyz=(link_2_len, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=zinc,
        name="tip_pivot_pin",
    )

    link_3 = model.part("link_3")
    link_3.visual(
        _flat_link_mesh("link_3_bar", length=link_3_len, width=link_width, thickness=link_thickness, hole_diameter=hole_diameter),
        origin=Origin(xyz=(0.0, -0.012, 0.0)),
        material=brushed_steel,
        name="slotted_bar",
    )
    link_3.visual(
        _pad_mesh("end_attachment_pad", link_length=link_3_len, pad_length=0.080, pad_height=0.058, thickness=link_thickness),
        origin=Origin(xyz=(0.0, -0.012, 0.0)),
        material=dark_steel,
        name="attachment_pad",
    )
    for x, label in ((0.0, "root"), (link_3_len, "tip")):
        link_3.visual(
            Cylinder(radius=cap_radius, length=cap_thickness),
            origin=Origin(xyz=(x, -0.0164, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
            material=zinc,
            name=f"{label}_rivet_cap",
        )

    # Rest pose is a compact zig-zag.  The same three pivots can be posed nearly
    # straight with the joint limits below, like a task-light stay.
    base_to_link_1 = model.articulation(
        "base_to_link_1",
        ArticulationType.REVOLUTE,
        parent=base_bracket,
        child=link_1,
        origin=Origin(rpy=(0.0, -0.65, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=4.0, lower=-0.55, upper=1.20),
    )
    link_1_to_link_2 = model.articulation(
        "link_1_to_link_2",
        ArticulationType.REVOLUTE,
        parent=link_1,
        child=link_2,
        origin=Origin(xyz=(link_1_len, 0.0, 0.0), rpy=(0.0, 2.05, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=4.0, lower=-2.10, upper=0.45),
    )
    link_2_to_link_3 = model.articulation(
        "link_2_to_link_3",
        ArticulationType.REVOLUTE,
        parent=link_2,
        child=link_3,
        origin=Origin(xyz=(link_2_len, 0.0, 0.0), rpy=(0.0, -2.00, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=4.0, lower=-0.45, upper=2.05),
    )

    # Names retained for readability when probing; suppress unused-variable linters
    # in standalone use without changing the authored model.
    _ = (base_to_link_1, link_1_to_link_2, link_2_to_link_3)
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_bracket")
    link_1 = object_model.get_part("link_1")
    link_2 = object_model.get_part("link_2")
    link_3 = object_model.get_part("link_3")
    joint_1 = object_model.get_articulation("base_to_link_1")
    joint_2 = object_model.get_articulation("link_1_to_link_2")
    joint_3 = object_model.get_articulation("link_2_to_link_3")

    pivots = (joint_1, joint_2, joint_3)
    ctx.check(
        "three revolute pivots",
        all(j.articulation_type == ArticulationType.REVOLUTE for j in pivots),
        details=f"joint types={[j.articulation_type for j in pivots]}",
    )
    ctx.check(
        "pivot axes are parallel",
        all(tuple(round(v, 6) for v in j.axis) == (0.0, 1.0, 0.0) for j in pivots),
        details=f"axes={[j.axis for j in pivots]}",
    )

    # Each pin is intentionally a captured axle through a bar hole, not a broad
    # body collision.  The exact projected checks prove the local axle/hole
    # relationship that the overlap allowance represents.
    ctx.allow_overlap(
        base,
        link_1,
        elem_a="base_pivot_pin",
        elem_b="slotted_bar",
        reason="The base pin is intentionally captured through the first link's pivot hole.",
    )
    ctx.expect_overlap(
        base,
        link_1,
        axes="xz",
        elem_a="base_pivot_pin",
        elem_b="slotted_bar",
        min_overlap=0.009,
        name="base pin passes through first link hole",
    )
    ctx.allow_overlap(
        link_1,
        link_2,
        elem_a="tip_pivot_pin",
        elem_b="slotted_bar",
        reason="The first link's tip pin is intentionally captured through the second link's pivot hole.",
    )
    ctx.expect_overlap(
        link_1,
        link_2,
        axes="xz",
        elem_a="tip_pivot_pin",
        elem_b="slotted_bar",
        min_overlap=0.009,
        name="middle pin passes through second link hole",
    )
    ctx.allow_overlap(
        link_2,
        link_3,
        elem_a="tip_pivot_pin",
        elem_b="slotted_bar",
        reason="The second link's tip pin is intentionally captured through the third link's pivot hole.",
    )
    ctx.expect_overlap(
        link_2,
        link_3,
        axes="xz",
        elem_a="tip_pivot_pin",
        elem_b="slotted_bar",
        min_overlap=0.009,
        name="outer pin passes through third link hole",
    )

    rest_aabb = ctx.part_world_aabb(link_3)

    def max_x(aabb):
        return None if aabb is None else aabb[1][0]

    with ctx.pose({joint_1: 0.65, joint_2: -2.05, joint_3: 2.00}):
        extended_aabb = ctx.part_world_aabb(link_3)
        ctx.expect_origin_gap(
            link_3,
            base,
            axis="x",
            min_gap=0.36,
            name="extended chain moves outward from bracket",
        )

    rest_tip_x = max_x(rest_aabb)
    extended_tip_x = max_x(extended_aabb)
    ctx.check(
        "folded pose stays compact",
        rest_tip_x is not None and rest_tip_x < 0.45,
        details=f"folded link_3 max_x={rest_tip_x}",
    )
    ctx.check(
        "extended pose reaches farther",
        rest_tip_x is not None and extended_tip_x is not None and extended_tip_x > rest_tip_x + 0.20,
        details=f"folded max_x={rest_tip_x}, extended max_x={extended_tip_x}",
    )
    return ctx.report()


object_model = build_object_model()
