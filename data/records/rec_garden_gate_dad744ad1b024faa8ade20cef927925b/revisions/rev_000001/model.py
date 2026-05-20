from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    wire_from_points,
)


IRON = Material("oiled_black_wrought_iron", rgba=(0.01, 0.009, 0.007, 1.0))
STONE = Material("warm_limestone", rgba=(0.55, 0.51, 0.44, 1.0))
DARK_MORTAR = Material("recessed_mortar", rgba=(0.18, 0.17, 0.15, 1.0))
WEATHERED_CAP = Material("weathered_capstone", rgba=(0.42, 0.40, 0.36, 1.0))


def _origin(x: float, y: float, z: float, rpy=(0.0, 0.0, 0.0)) -> Origin:
    return Origin(xyz=(x, y, z), rpy=rpy)


def _add_box(part, name: str, size, xyz, material) -> None:
    part.visual(Box(size), origin=_origin(*xyz), material=material, name=name)


def _add_cylinder(part, name: str, radius: float, length: float, xyz, material, rpy=(0.0, 0.0, 0.0)) -> None:
    part.visual(Cylinder(radius=radius, length=length), origin=_origin(*xyz, rpy=rpy), material=material, name=name)


def _add_leaf_frame(leaf) -> None:
    width = 0.76
    left_x = 0.060
    right_x = 0.740
    bottom_z = 0.135
    shoulder_z = 1.370
    rise = 0.230
    arch_points = []
    for i in range(17):
        t = i / 16.0
        x = left_x + (right_x - left_x) * t
        z = shoulder_z + rise * math.sin(math.pi * t)
        arch_points.append((x, 0.0, z))

    perimeter_points = [
        (left_x, 0.0, bottom_z),
        (left_x, 0.0, shoulder_z),
        *arch_points[1:-1],
        (right_x, 0.0, shoulder_z),
        (right_x, 0.0, bottom_z),
    ]
    perimeter = wire_from_points(
        perimeter_points,
        radius=0.018,
        radial_segments=20,
        closed_path=True,
        cap_ends=False,
        corner_mode="fillet",
        corner_radius=0.018,
    )
    leaf.visual(mesh_from_geometry(perimeter, "arched_perimeter_frame"), material=IRON, name="arched_perimeter")

    # Straight rails that make the narrow gate read as a forged iron leaf.
    _add_cylinder(leaf, "lower_rail", 0.013, 0.650, (0.400, 0.0, 0.330), IRON, rpy=(0.0, math.pi / 2.0, 0.0))
    _add_cylinder(leaf, "top_rail", 0.014, 0.620, (0.405, 0.0, 1.285), IRON, rpy=(0.0, math.pi / 2.0, 0.0))

    # Vertical infill bars, each slightly let into the bottom rail and the arch.
    for idx, x in enumerate([0.155, 0.255, 0.355, 0.455, 0.555, 0.655]):
        t = (x - left_x) / (right_x - left_x)
        top_z = shoulder_z + rise * math.sin(math.pi * t) - 0.018
        bottom = 0.315
        length = top_z - bottom
        _add_cylinder(
            leaf,
            f"vertical_bar_{idx}",
            0.0085,
            length,
            (x, 0.0, bottom + length / 2.0),
            IRON,
        )
        leaf.visual(
            Sphere(0.014),
            origin=_origin(x, 0.0, top_z + 0.002),
            material=IRON,
            name=f"bar_tip_{idx}",
        )

    # Hinge straps and the leaf-side hinge knuckles are part of the moving leaf.
    for idx, zc in enumerate([0.475, 1.295]):
        _add_box(leaf, f"hinge_strap_{idx}", (0.085, 0.028, 0.035), (0.0625, 0.0, zc), IRON)
        _add_cylinder(leaf, f"leaf_knuckle_{idx}", 0.020, 0.125, (0.0, 0.0, zc), IRON)

    # Latch hardware fixed to the meeting stile; the ring itself is a child part.
    _add_box(leaf, "latch_backplate", (0.060, 0.016, 0.115), (0.713, -0.018, 0.842), IRON)
    _add_cylinder(leaf, "latch_pivot_boss", 0.015, 0.030, (0.713, -0.038, 0.842), IRON, rpy=(math.pi / 2.0, 0.0, 0.0))

    # A protruding lug and pin for the upper closer arm near the top rail.
    _add_box(leaf, "closer_tab", (0.060, 0.215, 0.020), (0.460, -0.108, 1.302), IRON)
    _add_cylinder(leaf, "leaf_closer_pin", 0.018, 0.180, (0.460, -0.220, 1.390), IRON)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="wrought_iron_garden_gate",
        materials=[IRON, STONE, DARK_MORTAR, WEATHERED_CAP],
    )

    masonry = model.part("masonry")
    # A continuous threshold ties the two piers into one grounded masonry assembly.
    _add_box(masonry, "stone_threshold", (1.42, 0.42, 0.110), (0.0, 0.0, 0.055), STONE)
    for name, x in (("hinge_pier", -0.560), ("meeting_pier", 0.560)):
        _add_box(masonry, name, (0.250, 0.360, 1.900), (x, 0.0, 0.995), STONE)
        _add_box(masonry, f"{name}_plinth", (0.310, 0.430, 0.150), (x, 0.0, 0.185), WEATHERED_CAP)
        _add_box(masonry, f"{name}_cap", (0.330, 0.450, 0.120), (x, 0.0, 1.995), WEATHERED_CAP)

        # Shallow proud courses on the front face give the piers masonry scale.
        for course in range(9):
            z = 0.360 + course * 0.175
            offset = 0.0 if course % 2 == 0 else 0.047
            for brick in range(2):
                bx = x - 0.052 + brick * 0.104 + offset * (1 if brick == 0 else -1)
                _add_box(
                    masonry,
                    f"{name}_brick_{course}_{brick}",
                    (0.094, 0.010, 0.125),
                    (bx, -0.185, z),
                    STONE,
                )
            _add_box(
                masonry,
                f"{name}_mortar_{course}",
                (0.235, 0.014, 0.010),
                (x, -0.185, z + 0.070),
                DARK_MORTAR,
            )

    hinge_x = -0.390
    # Fixed-side hinge knuckles and straps on the inner face of the hinge pier.
    for idx, zc in enumerate([0.340, 0.610, 1.160, 1.430]):
        _add_box(masonry, f"fixed_hinge_strap_{idx}", (0.060, 0.034, 0.035), (-0.420, 0.0, zc), IRON)
        _add_cylinder(masonry, f"fixed_knuckle_{idx}", 0.020, 0.105, (hinge_x, 0.0, zc), IRON)
    _add_cylinder(masonry, "hinge_pin", 0.010, 1.300, (hinge_x, 0.0, 0.885), IRON)

    # Keeper and stop at the meeting pier.
    _add_box(masonry, "latch_keeper_plate", (0.030, 0.030, 0.150), (0.422, -0.020, 0.840), IRON)
    _add_cylinder(masonry, "latch_keeper_loop", 0.011, 0.080, (0.407, -0.048, 0.840), IRON, rpy=(math.pi / 2.0, 0.0, 0.0))

    # Fixed pivot pin and wall plate for the closer arm.
    _add_box(masonry, "closer_wall_plate", (0.070, 0.030, 0.090), (-0.460, -0.196, 1.335), IRON)
    _add_cylinder(masonry, "closer_wall_pin", 0.018, 0.135, (-0.460, -0.220, 1.400), IRON)

    leaf = model.part("leaf")
    _add_leaf_frame(leaf)

    ring = model.part("ring_latch")
    ring.visual(
        mesh_from_geometry(TorusGeometry(0.042, 0.0055, radial_segments=18, tubular_segments=42), "ring_latch"),
        origin=_origin(0.0, 0.0, -0.060, rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=IRON,
        name="pull_ring",
    )
    _add_cylinder(ring, "ring_pivot_pin", 0.006, 0.026, (0.0, 0.0, 0.0), IRON, rpy=(0.0, math.pi / 2.0, 0.0))
    _add_cylinder(ring, "ring_neck", 0.0045, 0.050, (0.0, 0.0, -0.025), IRON)

    closer_arm = model.part("closer_arm")
    closer_arm.visual(
        mesh_from_geometry(TorusGeometry(0.023, 0.006, radial_segments=18, tubular_segments=36), "closer_wall_eye"),
        material=IRON,
        name="wall_eye",
    )
    _add_box(closer_arm, "flat_arm", (0.474, 0.024, 0.012), (0.265, 0.0, 0.0), IRON)

    arm_end = model.part("arm_end")
    arm_end.visual(
        mesh_from_geometry(TorusGeometry(0.023, 0.006, radial_segments=18, tubular_segments=36), "closer_leaf_eye"),
        material=IRON,
        name="leaf_eye",
    )
    _add_box(arm_end, "end_tab", (0.066, 0.016, 0.010), (0.061, 0.0, 0.0), IRON)

    model.articulation(
        "leaf_hinge",
        ArticulationType.REVOLUTE,
        parent=masonry,
        child=leaf,
        origin=_origin(hinge_x, 0.0, 0.0),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=90.0, velocity=1.2, lower=0.0, upper=1.45),
    )
    model.articulation(
        "ring_pivot",
        ArticulationType.REVOLUTE,
        parent=leaf,
        child=ring,
        origin=_origin(0.713, -0.055, 0.842),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=4.0, lower=-0.75, upper=0.75),
    )
    model.articulation(
        "closer_wall_pivot",
        ArticulationType.REVOLUTE,
        parent=masonry,
        child=closer_arm,
        origin=_origin(-0.460, -0.220, 1.400),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=1.5, lower=-0.45, upper=0.65),
    )
    model.articulation(
        "closer_leaf_pivot",
        ArticulationType.REVOLUTE,
        parent=closer_arm,
        child=arm_end,
        origin=_origin(0.530, 0.0, 0.0),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.0, lower=-1.1, upper=1.1),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    masonry = object_model.get_part("masonry")
    leaf = object_model.get_part("leaf")
    ring = object_model.get_part("ring_latch")
    closer_arm = object_model.get_part("closer_arm")
    arm_end = object_model.get_part("arm_end")
    leaf_hinge = object_model.get_articulation("leaf_hinge")
    ring_pivot = object_model.get_articulation("ring_pivot")
    wall_pivot = object_model.get_articulation("closer_wall_pivot")
    leaf_pivot = object_model.get_articulation("closer_leaf_pivot")

    # Local captured-pin overlaps intentionally replace hollow hinge/eye bores
    # with solid proxy geometry.  Each is scoped to the specific pin-and-eye
    # element that creates the mechanical support path.
    ctx.allow_overlap(
        leaf,
        masonry,
        elem_a="leaf_knuckle_0",
        elem_b="hinge_pin",
        reason="The leaf knuckle is intentionally captured around the masonry-side hinge pin.",
    )
    ctx.allow_overlap(
        leaf,
        masonry,
        elem_a="leaf_knuckle_1",
        elem_b="hinge_pin",
        reason="The upper leaf knuckle is intentionally captured around the same hinge pin.",
    )
    ctx.expect_within(
        masonry,
        leaf,
        axes="xy",
        inner_elem="hinge_pin",
        outer_elem="leaf_knuckle_0",
        margin=0.001,
        name="lower hinge pin is centered in leaf knuckle",
    )
    ctx.expect_overlap(
        leaf,
        masonry,
        axes="z",
        elem_a="leaf_knuckle_0",
        elem_b="hinge_pin",
        min_overlap=0.10,
        name="lower leaf knuckle has retained hinge insertion",
    )

    ctx.allow_overlap(
        closer_arm,
        masonry,
        elem_a="wall_eye",
        elem_b="closer_wall_pin",
        reason="The closer arm eye is modeled as a solid proxy captured by its wall pivot pin.",
    )
    ctx.expect_within(
        masonry,
        closer_arm,
        axes="xy",
        inner_elem="closer_wall_pin",
        outer_elem="wall_eye",
        margin=0.001,
        name="wall closer pin sits within arm eye",
    )
    ctx.expect_overlap(
        closer_arm,
        masonry,
        axes="z",
        elem_a="wall_eye",
        elem_b="closer_wall_pin",
        min_overlap=0.010,
        name="wall closer eye is vertically retained on pin",
    )

    ctx.allow_overlap(
        arm_end,
        leaf,
        elem_a="leaf_eye",
        elem_b="leaf_closer_pin",
        reason="The outer closer eye is intentionally captured around the gate-leaf pivot pin.",
    )
    ctx.expect_within(
        leaf,
        arm_end,
        axes="xy",
        inner_elem="leaf_closer_pin",
        outer_elem="leaf_eye",
        margin=0.001,
        name="leaf closer pin sits within outer arm eye",
    )
    ctx.expect_overlap(
        arm_end,
        leaf,
        axes="z",
        elem_a="leaf_eye",
        elem_b="leaf_closer_pin",
        min_overlap=0.010,
        name="outer closer eye is vertically retained on pin",
    )

    closed_leaf = ctx.part_world_aabb(leaf)
    with ctx.pose({leaf_hinge: 1.0}):
        opened_leaf = ctx.part_world_aabb(leaf)
    ctx.check(
        "leaf opens outward about vertical hinge",
        closed_leaf is not None
        and opened_leaf is not None
        and opened_leaf[1][1] > closed_leaf[1][1] + 0.20,
        details=f"closed={closed_leaf}, opened={opened_leaf}",
    )

    ring_closed = ctx.part_element_world_aabb(ring, elem="pull_ring")
    with ctx.pose({ring_pivot: 0.70}):
        ring_lifted = ctx.part_element_world_aabb(ring, elem="pull_ring")
    ctx.check(
        "ring latch swings on small horizontal pivot",
        ring_closed is not None
        and ring_lifted is not None
        and (ring_lifted[1][1] - ring_lifted[0][1]) > (ring_closed[1][1] - ring_closed[0][1]) + 0.020,
        details=f"closed={ring_closed}, lifted={ring_lifted}",
    )

    closed_end_position = ctx.part_world_position(arm_end)
    with ctx.pose({wall_pivot: 0.55}):
        swung_end_position = ctx.part_world_position(arm_end)
    ctx.check(
        "closer arm rotates at wall support pivot",
        closed_end_position is not None
        and swung_end_position is not None
        and swung_end_position[1] > closed_end_position[1] + 0.20,
        details=f"closed={closed_end_position}, swung={swung_end_position}",
    )

    end_tab_closed = ctx.part_element_world_aabb(arm_end, elem="end_tab")
    with ctx.pose({leaf_pivot: 0.95}):
        end_tab_rotated = ctx.part_element_world_aabb(arm_end, elem="end_tab")
    ctx.check(
        "closer arm end rotates at leaf support pivot",
        end_tab_closed is not None
        and end_tab_rotated is not None
        and (end_tab_rotated[1][1] - end_tab_rotated[0][1]) > (end_tab_closed[1][1] - end_tab_closed[0][1]) + 0.030,
        details=f"closed={end_tab_closed}, rotated={end_tab_rotated}",
    )

    return ctx.report()


object_model = build_object_model()
