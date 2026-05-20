from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def _bar_xz(
    part,
    *,
    name: str,
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    size_y: float,
    size_z: float,
    material,
) -> None:
    """Rectangular member whose long local X axis runs between two XZ points."""

    sx = end[0] - start[0]
    sy = end[1] - start[1]
    sz = end[2] - start[2]
    length = math.sqrt(sx * sx + sy * sy + sz * sz)
    # All ladder structural members in this model lie in XZ planes, so a
    # pitch-only transform keeps their rectangular faces clean and readable.
    pitch = math.atan2(-sz, sx)
    part.visual(
        Box((length, size_y, size_z)),
        origin=Origin(
            xyz=((start[0] + end[0]) * 0.5, (start[1] + end[1]) * 0.5, (start[2] + end[2]) * 0.5),
            rpy=(0.0, pitch, 0.0),
        ),
        material=material,
        name=name,
    )


def _cylinder_y(part, *, name: str, radius: float, length: float, xyz, material) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def _cylinder_x(part, *, name: str, radius: float, length: float, xyz, material) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(0.0, math.pi / 2.0, 0.0)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_a_frame_step_ladder")

    satin_aluminum = model.material("satin_warm_aluminum", rgba=(0.74, 0.70, 0.63, 1.0))
    soft_graphite = model.material("matte_graphite", rgba=(0.075, 0.080, 0.085, 1.0))
    black_rubber = model.material("soft_black_rubber", rgba=(0.012, 0.012, 0.011, 1.0))
    dark_plastic = model.material("charcoal_polymer", rgba=(0.035, 0.039, 0.043, 1.0))
    hardware = model.material("brushed_dark_steel", rgba=(0.36, 0.35, 0.33, 1.0))
    seam_shadow = model.material("recess_shadow", rgba=(0.006, 0.007, 0.008, 1.0))

    top_deck = model.part("top_deck")
    top_deck.visual(
        Box((0.38, 0.74, 0.055)),
        origin=Origin(xyz=(0.035, 0.0, 1.445)),
        material=dark_plastic,
        name="platform_shell",
    )
    # A recessed, non-slip top surface breaks up the plastic platform without
    # making it look like a crude solid block.
    top_deck.visual(
        Box((0.285, 0.57, 0.008)),
        origin=Origin(xyz=(0.050, 0.0, 1.4765)),
        material=soft_graphite,
        name="recessed_pad",
    )
    top_deck.visual(
        Box((0.400, 0.036, 0.040)),
        origin=Origin(xyz=(0.035, 0.372, 1.475)),
        material=soft_graphite,
        name="side_rim_0",
    )
    top_deck.visual(
        Box((0.400, 0.036, 0.040)),
        origin=Origin(xyz=(0.035, -0.372, 1.475)),
        material=soft_graphite,
        name="side_rim_1",
    )
    top_deck.visual(
        Box((0.036, 0.74, 0.038)),
        origin=Origin(xyz=(-0.170, 0.0, 1.472)),
        material=soft_graphite,
        name="front_lip",
    )
    top_deck.visual(
        Box((0.030, 0.62, 0.032)),
        origin=Origin(xyz=(0.239, 0.0, 1.472)),
        material=soft_graphite,
        name="rear_lip",
    )
    top_deck.visual(
        Box((0.165, 0.405, 0.006)),
        origin=Origin(xyz=(0.062, 0.0, 1.4755)),
        material=seam_shadow,
        name="fine_recess_seam",
    )
    for idx, y in enumerate((-0.235, -0.118, 0.0, 0.118, 0.235)):
        top_deck.visual(
            Box((0.230, 0.010, 0.007)),
            origin=Origin(xyz=(0.058, y, 1.484)),
            material=black_rubber,
            name=f"top_grip_rib_{idx}",
        )
    # Hinge spine and restrained pivot hardware are part of the fixed top deck.
    _cylinder_y(
        top_deck,
        name="hinge_spine",
        radius=0.018,
        length=0.790,
        xyz=(0.0, 0.0, 1.405),
        material=hardware,
    )
    for idx, y in enumerate((-0.405, 0.405)):
        top_deck.visual(
            Box((0.110, 0.026, 0.115)),
            origin=Origin(xyz=(0.015, y, 1.365)),
            material=hardware,
            name=f"hinge_cheek_{idx}",
        )
        _cylinder_y(
            top_deck,
            name=f"pin_head_{idx}",
            radius=0.026,
            length=0.010,
            xyz=(0.0, y * 0.988, 1.405),
            material=hardware,
        )

    front_frame = model.part("front_frame")
    # Side rails: warm satin aluminum rectangular tubes, splayed slightly at the
    # floor for stability and joined by practical, full-width treads.
    for idx, y in enumerate((-0.305, 0.305)):
        _bar_xz(
            front_frame,
            name=f"front_rail_{idx}",
            start=(-0.325, y, -1.375),
            end=(-0.020, y, -0.055),
            size_y=0.043,
            size_z=0.038,
            material=satin_aluminum,
        )
        front_frame.visual(
            Box((0.100, 0.072, 0.032)),
            origin=Origin(xyz=(-0.342, y, -1.390), rpy=(0.0, 0.12, 0.0)),
            material=black_rubber,
            name=f"front_foot_{idx}",
        )
        front_frame.visual(
            Box((0.088, 0.056, 0.090)),
            origin=Origin(xyz=(-0.022, y + (0.047 if y > 0 else -0.047), -0.070)),
            material=hardware,
            name=f"upper_knuckle_{idx}",
        )
        front_frame.visual(
            Box((0.085, 0.030, 0.074)),
            origin=Origin(xyz=(-0.220, y + (0.040 if y > 0 else -0.040), -0.820)),
            material=hardware,
            name=f"brace_lug_{idx}",
        )
    _cylinder_y(
        front_frame,
        name="top_cross_tube",
        radius=0.015,
        length=0.610,
        xyz=(-0.035, 0.0, -0.070),
        material=satin_aluminum,
    )
    _cylinder_y(
        front_frame,
        name="lower_cross_tube",
        radius=0.012,
        length=0.610,
        xyz=(-0.280, 0.0, -1.170),
        material=satin_aluminum,
    )
    tread_specs = [
        ("tread_0", -1.065, -0.250, 0.195),
        ("tread_1", -0.755, -0.180, 0.185),
        ("tread_2", -0.445, -0.108, 0.175),
    ]
    for tread_name, z, x, depth in tread_specs:
        front_frame.visual(
            Box((depth, 0.650, 0.038)),
            origin=Origin(xyz=(x, 0.0, z)),
            material=satin_aluminum,
            name=tread_name,
        )
        front_frame.visual(
            Box((0.030, 0.662, 0.048)),
            origin=Origin(xyz=(x - depth * 0.5 + 0.014, 0.0, z + 0.010)),
            material=hardware,
            name=f"{tread_name}_nosing",
        )
        front_frame.visual(
            Box((depth - 0.052, 0.565, 0.006)),
            origin=Origin(xyz=(x + 0.010, 0.0, z + 0.022)),
            material=soft_graphite,
            name=f"{tread_name}_insert",
        )
        for rib_idx, dx in enumerate((-0.052, -0.026, 0.0, 0.026, 0.052)):
            front_frame.visual(
                Box((0.007, 0.535, 0.010)),
                origin=Origin(xyz=(x + dx, 0.0, z + 0.030)),
                material=black_rubber,
                name=f"{tread_name}_rib_{rib_idx}",
            )
    # Small side gussets and exposed rivets clarify that the treads are carried
    # by the rails rather than floating between them.
    for idx, y in enumerate((-0.334, 0.334)):
        for gusset_idx, (x, z) in enumerate(((-0.250, -1.065), (-0.180, -0.755), (-0.108, -0.445))):
            front_frame.visual(
                Box((0.070, 0.018, 0.062)),
                origin=Origin(xyz=(x - 0.030, y, z + 0.002), rpy=(0.0, 0.10, 0.0)),
                material=hardware,
                name=f"tread_gusset_{idx}_{gusset_idx}",
            )
            _cylinder_y(
                front_frame,
                name=f"front_rivet_{idx}_{gusset_idx}",
                radius=0.008,
                length=0.007,
                xyz=(x - 0.056, y + (0.010 if y > 0 else -0.010), z + 0.018),
                material=hardware,
            )

    rear_frame = model.part("rear_frame")
    for idx, y in enumerate((-0.305, 0.305)):
        _bar_xz(
            rear_frame,
            name=f"rear_rail_{idx}",
            start=(0.555, y, -1.375),
            end=(0.050, y, -0.055),
            size_y=0.040,
            size_z=0.036,
            material=satin_aluminum,
        )
        rear_frame.visual(
            Box((0.104, 0.070, 0.032)),
            origin=Origin(xyz=(0.575, y, -1.390), rpy=(0.0, -0.15, 0.0)),
            material=black_rubber,
            name=f"rear_foot_{idx}",
        )
        rear_frame.visual(
            Box((0.086, 0.052, 0.082)),
            origin=Origin(xyz=(0.061, y * 0.84, -0.030)),
            material=hardware,
            name=f"rear_knuckle_{idx}",
        )
        # Bracket/slot that receives the spread-limit brace end.
        rear_frame.visual(
            Box((0.090, 0.055, 0.050)),
            origin=Origin(xyz=(0.370, y + (0.040 if y > 0 else -0.040), -0.860)),
            material=hardware,
            name=f"brace_slot_{idx}",
        )
    _cylinder_y(
        rear_frame,
        name="rear_top_cross",
        radius=0.014,
        length=0.610,
        xyz=(0.048, 0.0, -0.075),
        material=satin_aluminum,
    )
    _cylinder_y(
        rear_frame,
        name="rear_mid_cross",
        radius=0.012,
        length=0.590,
        xyz=(0.318, 0.0, -0.740),
        material=satin_aluminum,
    )
    _cylinder_y(
        rear_frame,
        name="rear_low_cross",
        radius=0.012,
        length=0.590,
        xyz=(0.448, 0.0, -1.075),
        material=satin_aluminum,
    )
    _bar_xz(
        rear_frame,
        name="rear_diagonal_strut_0",
        start=(0.135, -0.286, -0.270),
        end=(0.480, -0.286, -1.160),
        size_y=0.022,
        size_z=0.020,
        material=satin_aluminum,
    )
    _bar_xz(
        rear_frame,
        name="rear_diagonal_strut_1",
        start=(0.135, 0.286, -0.270),
        end=(0.480, 0.286, -1.160),
        size_y=0.022,
        size_z=0.020,
        material=satin_aluminum,
    )

    for side_idx, y in enumerate((-0.343, 0.343)):
        brace = model.part(f"brace_{side_idx}")
        _bar_xz(
            brace,
            name="flat_spreader",
            start=(0.0, 0.0, 0.0),
            end=(0.592, 0.0, -0.042),
            size_y=0.018,
            size_z=0.020,
            material=hardware,
        )
        _cylinder_y(
            brace,
            name="front_pivot_eye",
            radius=0.025,
            length=0.020,
            xyz=(0.0, 0.0, 0.0),
            material=hardware,
        )
        _cylinder_y(
            brace,
            name="rear_slider_pin",
            radius=0.018,
            length=0.018,
            xyz=(0.592, 0.0, -0.042),
            material=hardware,
        )
        brace.visual(
            Box((0.070, 0.022, 0.030)),
            origin=Origin(xyz=(0.313, 0.0, -0.022)),
            material=seam_shadow,
            name="center_lock_seam",
        )
        model.articulation(
            f"front_to_brace_{side_idx}",
            ArticulationType.REVOLUTE,
            parent=front_frame,
            child=brace,
            origin=Origin(xyz=(-0.220, y, -0.820)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=12.0, velocity=1.4, lower=0.0, upper=0.36),
            mimic=Mimic(joint="top_to_rear", multiplier=0.90, offset=0.0),
        )

    model.articulation(
        "top_to_front",
        ArticulationType.REVOLUTE,
        parent=top_deck,
        child=front_frame,
        origin=Origin(xyz=(0.0, 0.0, 1.405)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=1.0, lower=-0.34, upper=0.03),
    )
    model.articulation(
        "top_to_rear",
        ArticulationType.REVOLUTE,
        parent=top_deck,
        child=rear_frame,
        origin=Origin(xyz=(0.0, 0.0, 1.405)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=1.0, lower=0.0, upper=0.40),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    front = object_model.get_part("front_frame")
    rear = object_model.get_part("rear_frame")
    top = object_model.get_part("top_deck")
    front_hinge = object_model.get_articulation("top_to_front")
    rear_hinge = object_model.get_articulation("top_to_rear")

    ctx.expect_overlap(
        front,
        top,
        axes="y",
        elem_a="top_cross_tube",
        elem_b="hinge_spine",
        min_overlap=0.55,
        name="front frame shares the full-width hinge line",
    )
    ctx.expect_overlap(
        rear,
        top,
        axes="y",
        elem_a="rear_top_cross",
        elem_b="hinge_spine",
        min_overlap=0.55,
        name="rear frame shares the full-width hinge line",
    )
    ctx.expect_gap(
        top,
        front,
        axis="z",
        positive_elem="platform_shell",
        negative_elem="tread_2",
        min_gap=0.34,
        name="top platform sits well above the upper climbing tread",
    )

    front_foot_open = ctx.part_element_world_aabb(front, elem="front_foot_0")
    rear_foot_open = ctx.part_element_world_aabb(rear, elem="rear_foot_0")
    if front_foot_open is not None and rear_foot_open is not None:
        open_span = rear_foot_open[1][0] - front_foot_open[0][0]
    else:
        open_span = None

    with ctx.pose({front_hinge: -0.30, rear_hinge: 0.36}):
        front_foot_folded = ctx.part_element_world_aabb(front, elem="front_foot_0")
        rear_foot_folded = ctx.part_element_world_aabb(rear, elem="rear_foot_0")
        if front_foot_folded is not None and rear_foot_folded is not None:
            folded_span = rear_foot_folded[1][0] - front_foot_folded[0][0]
        else:
            folded_span = None

    ctx.check(
        "hinged frames fold toward a compact stance",
        open_span is not None and folded_span is not None and folded_span < open_span - 0.42,
        details=f"open_span={open_span}, folded_span={folded_span}",
    )

    for side_idx in (0, 1):
        brace = object_model.get_part(f"brace_{side_idx}")
        ctx.allow_overlap(
            brace,
            front,
            elem_a="front_pivot_eye",
            elem_b=f"brace_lug_{side_idx}",
            reason="The spreader pivot eye is intentionally captured through the front clevis lug proxy.",
        )
        ctx.allow_overlap(
            brace,
            front,
            elem_a="flat_spreader",
            elem_b=f"brace_lug_{side_idx}",
            reason="The flat spreader strap locally enters the front lug so the visible pivot is seated, not floating.",
        )
        ctx.allow_overlap(
            brace,
            rear,
            elem_a="flat_spreader",
            elem_b=f"brace_slot_{side_idx}",
            reason="The end of the spread-limit bar is intentionally seated inside the rear stop-slot proxy.",
        )
        ctx.allow_overlap(
            brace,
            rear,
            elem_a="rear_slider_pin",
            elem_b=f"brace_slot_{side_idx}",
            reason="The rear brace pin is intentionally captured inside the stop-slot bracket.",
        )
        ctx.expect_overlap(
            brace,
            front,
            axes="xz",
            elem_a="front_pivot_eye",
            elem_b=f"brace_lug_{side_idx}",
            min_overlap=0.010,
            name=f"brace {side_idx} is pinned at the climbing rail",
        )
        ctx.expect_overlap(
            brace,
            rear,
            axes="xz",
            elem_a="rear_slider_pin",
            elem_b=f"brace_slot_{side_idx}",
            min_overlap=0.010,
            name=f"brace {side_idx} reaches the rear spread-stop slot",
        )
        ctx.expect_overlap(
            brace,
            front,
            axes="xz",
            elem_a="flat_spreader",
            elem_b=f"brace_lug_{side_idx}",
            min_overlap=0.018,
            name=f"brace {side_idx} strap enters the front pivot lug",
        )
        ctx.expect_overlap(
            brace,
            rear,
            axes="xz",
            elem_a="flat_spreader",
            elem_b=f"brace_slot_{side_idx}",
            min_overlap=0.020,
            name=f"brace {side_idx} flat bar is captured in the stop slot",
        )

    return ctx.report()


object_model = build_object_model()
