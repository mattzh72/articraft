from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _cyl_x(xyz: tuple[float, float, float]) -> Origin:
    """Origin for a cylinder whose local Z axis should run along world X."""
    return Origin(xyz=xyz, rpy=(0.0, math.pi / 2.0, 0.0))


def _cyl_y(xyz: tuple[float, float, float]) -> Origin:
    """Origin for a cylinder whose local Z axis should run along world Y."""
    return Origin(xyz=xyz, rpy=(-math.pi / 2.0, 0.0, 0.0))


def _add_optical_half(part, *, side: float, rubber: Material, armor: Material, glass: Material, steel: Material) -> None:
    """Add one connected porro-prism binocular half in a hinge-centered frame."""
    y = 0.102 * side
    prefix = "left" if side > 0.0 else "right"

    # Interleaved knuckles around the central hinge pin.  They deliberately
    # live at y=0 so the barrel half has an obvious mechanical pivot.
    if side > 0.0:
        part.visual(
            Cylinder(radius=0.020, length=0.075),
            origin=_cyl_x((-0.105, 0.0, 0.0)),
            material=steel,
            name="left_hinge_sleeve_0",
        )
        part.visual(
            Box((0.052, 0.074, 0.022)),
            origin=Origin(xyz=(-0.105, 0.050 * side, 0.006)),
            material=armor,
            name="left_hinge_arm_0",
        )
        part.visual(
            Cylinder(radius=0.020, length=0.075),
            origin=_cyl_x((0.100, 0.0, 0.0)),
            material=steel,
            name="left_hinge_sleeve_1",
        )
        part.visual(
            Box((0.052, 0.074, 0.022)),
            origin=Origin(xyz=(0.100, 0.050 * side, 0.006)),
            material=armor,
            name="left_hinge_arm_1",
        )
    else:
        part.visual(
            Cylinder(radius=0.020, length=0.105),
            origin=_cyl_x((-0.010, 0.0, 0.0)),
            material=steel,
            name="right_hinge_sleeve_0",
        )
        part.visual(
            Box((0.052, 0.074, 0.022)),
            origin=Origin(xyz=(-0.010, 0.050 * side, 0.006)),
            material=armor,
            name="right_hinge_arm_0",
        )

    # The blocky prism housing is the signature porro-prism mass behind the
    # objective tube; overlapping tube/housing/ribs make it one supported half.
    part.visual(
        Box((0.178, 0.082, 0.086)),
        origin=Origin(xyz=(-0.030, y, 0.018)),
        material=armor,
        name=f"{prefix}_prism_housing",
    )
    part.visual(
        Box((0.128, 0.070, 0.026)),
        origin=Origin(xyz=(-0.035, y, 0.073)),
        material=rubber,
        name=f"{prefix}_raised_prism_cap",
    )

    # Wide night-observation objective barrel, rubber armored, with a recessed
    # green-coated glass objective visible behind the cap when it flips open.
    part.visual(
        Cylinder(radius=0.052, length=0.260),
        origin=_cyl_x((0.082, y, 0.000)),
        material=rubber,
        name=f"{prefix}_objective_tube",
    )
    part.visual(
        Cylinder(radius=0.066, length=0.052),
        origin=_cyl_x((0.194, y, 0.000)),
        material=armor,
        name=f"{prefix}_objective_rim",
    )
    for ring_x, ring_name in ((0.040, "rear_rib"), (0.126, "front_rib")):
        part.visual(
            Cylinder(radius=0.056, length=0.016),
            origin=_cyl_x((ring_x, y, 0.000)),
            material=armor,
            name=f"{prefix}_{ring_name}",
        )
    if side > 0.0:
        part.visual(
            Cylinder(radius=0.047, length=0.004),
            origin=_cyl_x((0.221, y, 0.000)),
            material=glass,
            name="left_objective_glass",
        )
    else:
        part.visual(
            Cylinder(radius=0.047, length=0.004),
            origin=_cyl_x((0.221, y, 0.000)),
            material=glass,
            name="right_objective_glass",
        )

    # Rear eyepiece and soft eye cup.
    part.visual(
        Cylinder(radius=0.032, length=0.086),
        origin=_cyl_x((-0.148, y, 0.028)),
        material=rubber,
        name=f"{prefix}_eyepiece_tube",
    )
    part.visual(
        Cylinder(radius=0.038, length=0.028),
        origin=_cyl_x((-0.197, y, 0.028)),
        material=armor,
        name=f"{prefix}_eyecup",
    )
    part.visual(
        Cylinder(radius=0.024, length=0.003),
        origin=_cyl_x((-0.212, y, 0.028)),
        material=glass,
        name=f"{prefix}_eyepiece_glass",
    )

    # Fork ears for the flip-up objective cap hinge.  The cap's pin end faces
    # touch the inner faces of these ears at q=0, avoiding a floating cap.
    lug_offset = 0.031
    for sign, name in ((-1.0, "inner"), (1.0, "outer")):
        part.visual(
            Box((0.024, 0.012, 0.020)),
            origin=Origin(xyz=(0.224, y + sign * lug_offset, 0.072)),
            material=steel,
            name=f"{prefix}_cap_{name}_lug",
        )
    part.visual(
        Box((0.032, 0.064, 0.012)),
        origin=Origin(xyz=(0.198, y, 0.062)),
        material=armor,
        name=f"{prefix}_cap_hinge_saddle",
    )


def _add_lens_cap(part, *, side: float, rubber: Material, steel: Material) -> None:
    prefix = "left" if side > 0.0 else "right"
    part.visual(
        Cylinder(radius=0.006, length=0.050),
        origin=_cyl_y((0.0, 0.0, 0.0)),
        material=steel,
        name=f"{prefix}_cap_pin",
    )
    part.visual(
        Box((0.012, 0.020, 0.052)),
        origin=Origin(xyz=(0.006, 0.0, -0.030)),
        material=rubber,
        name=f"{prefix}_cap_strap",
    )
    if side > 0.0:
        part.visual(
            Cylinder(radius=0.064, length=0.010),
            origin=Origin(xyz=(0.008, 0.0, -0.078), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=rubber,
            name="left_cap_disk",
        )
    else:
        part.visual(
            Cylinder(radius=0.064, length=0.010),
            origin=Origin(xyz=(0.008, 0.0, -0.078), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=rubber,
            name="right_cap_disk",
        )
    part.visual(
        Cylinder(radius=0.052, length=0.003),
        origin=Origin(xyz=(0.014, 0.0, -0.078), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name=f"{prefix}_cap_inner_plate",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="night_vision_porro_binocular")

    rubber = model.material("matte_black_rubber", rgba=(0.015, 0.017, 0.016, 1.0))
    armor = model.material("olive_rubber_armor", rgba=(0.105, 0.135, 0.090, 1.0))
    steel = model.material("blackened_steel", rgba=(0.060, 0.062, 0.060, 1.0))
    glass = model.material("green_coated_glass", rgba=(0.080, 0.330, 0.190, 0.62))
    faint_red = model.material("dark_ir_lens", rgba=(0.390, 0.035, 0.025, 0.82))

    bridge = model.part("bridge")
    bridge.visual(
        Cylinder(radius=0.012, length=0.325),
        origin=_cyl_x((0.0, 0.0, 0.0)),
        material=steel,
        name="central_hinge_pin",
    )
    bridge.visual(
        Box((0.026, 0.026, 0.036)),
        origin=Origin(xyz=(-0.157, 0.0, 0.030)),
        material=steel,
        name="focus_stem",
    )
    bridge.visual(
        Box((0.026, 0.080, 0.012)),
        origin=Origin(xyz=(-0.157, 0.0, 0.018)),
        material=steel,
        name="focus_lower_yoke",
    )
    for y, name in ((-0.035, "focus_post_neg"), (0.035, "focus_post_pos")):
        bridge.visual(
            Box((0.026, 0.010, 0.034)),
            origin=Origin(xyz=(-0.157, y, 0.029)),
            material=steel,
            name=name,
        )
    bridge.visual(
        Box((0.078, 0.084, 0.010)),
        origin=Origin(xyz=(-0.157, 0.0, 0.104)),
        material=steel,
        name="focus_yoke_top",
    )
    bridge.visual(
        Box((0.044, 0.010, 0.060)),
        origin=Origin(xyz=(-0.157, -0.035, 0.074)),
        material=steel,
        name="focus_yoke_neg",
    )
    bridge.visual(
        Box((0.044, 0.010, 0.060)),
        origin=Origin(xyz=(-0.157, 0.035, 0.074)),
        material=steel,
        name="focus_yoke_pos",
    )
    bridge.visual(
        Box((0.026, 0.026, 0.050)),
        origin=Origin(xyz=(0.155, 0.0, 0.036)),
        material=steel,
        name="ir_support_post",
    )
    bridge.visual(
        Box((0.066, 0.040, 0.026)),
        origin=Origin(xyz=(0.155, 0.0, 0.072)),
        material=armor,
        name="ir_mount",
    )
    bridge.visual(
        Cylinder(radius=0.016, length=0.040),
        origin=_cyl_x((0.191, 0.0, 0.083)),
        material=steel,
        name="ir_illuminator_body",
    )
    bridge.visual(
        Cylinder(radius=0.012, length=0.003),
        origin=_cyl_x((0.2125, 0.0, 0.083)),
        material=faint_red,
        name="ir_front_lens",
    )

    left_body = model.part("left_body")
    _add_optical_half(left_body, side=1.0, rubber=rubber, armor=armor, glass=glass, steel=steel)

    right_body = model.part("right_body")
    _add_optical_half(right_body, side=-1.0, rubber=rubber, armor=armor, glass=glass, steel=steel)

    focus_wheel = model.part("focus_wheel")
    focus_mesh = mesh_from_geometry(
        KnobGeometry(
            0.052,
            0.048,
            body_style="cylindrical",
            edge_radius=0.0012,
            grip=KnobGrip(style="ribbed", count=28, depth=0.0014, width=0.0016),
        ),
        "center_focus_wheel",
    )
    focus_wheel.visual(
        focus_mesh,
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="ribbed_wheel",
    )
    focus_wheel.visual(
        Cylinder(radius=0.010, length=0.060),
        origin=_cyl_y((0.0, 0.0, 0.0)),
        material=steel,
        name="wheel_hub",
    )

    left_cap = model.part("left_cap")
    _add_lens_cap(left_cap, side=1.0, rubber=rubber, steel=steel)

    right_cap = model.part("right_cap")
    _add_lens_cap(right_cap, side=-1.0, rubber=rubber, steel=steel)

    model.articulation(
        "bridge_to_left_body",
        ArticulationType.REVOLUTE,
        parent=bridge,
        child=left_body,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=0.8, lower=-0.14, upper=0.16),
    )
    model.articulation(
        "bridge_to_right_body",
        ArticulationType.REVOLUTE,
        parent=bridge,
        child=right_body,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=0.8, lower=-0.16, upper=0.14),
    )
    model.articulation(
        "bridge_to_focus_wheel",
        ArticulationType.REVOLUTE,
        parent=bridge,
        child=focus_wheel,
        origin=Origin(xyz=(-0.157, 0.0, 0.074)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.8, velocity=8.0, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "left_body_to_left_cap",
        ArticulationType.REVOLUTE,
        parent=left_body,
        child=left_cap,
        origin=Origin(xyz=(0.224, 0.102, 0.072)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=0.5, velocity=4.0, lower=0.0, upper=2.15),
    )
    model.articulation(
        "right_body_to_right_cap",
        ArticulationType.REVOLUTE,
        parent=right_body,
        child=right_cap,
        origin=Origin(xyz=(0.224, -0.102, 0.072)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=0.5, velocity=4.0, lower=0.0, upper=2.15),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bridge = object_model.get_part("bridge")
    left_body = object_model.get_part("left_body")
    right_body = object_model.get_part("right_body")
    focus_wheel = object_model.get_part("focus_wheel")
    left_cap = object_model.get_part("left_cap")
    right_cap = object_model.get_part("right_cap")

    left_hinge = object_model.get_articulation("bridge_to_left_body")
    focus_joint = object_model.get_articulation("bridge_to_focus_wheel")
    left_cap_joint = object_model.get_articulation("left_body_to_left_cap")
    right_cap_joint = object_model.get_articulation("right_body_to_right_cap")

    # The central hinge sleeves are simplified as solid knuckles riding on a
    # solid blackened pin, so this is a small, local, mechanically intentional
    # overlap rather than an accidental body collision.
    for elem in ("left_hinge_sleeve_0", "left_hinge_sleeve_1"):
        ctx.allow_overlap(
            bridge,
            left_body,
            elem_a="central_hinge_pin",
            elem_b=elem,
            reason="The porro body knuckle is represented as a solid sleeve captured on the central hinge pin.",
        )
        ctx.expect_overlap(
            bridge,
            left_body,
            elem_a="central_hinge_pin",
            elem_b=elem,
            axes="x",
            min_overlap=0.060,
            name=f"{elem} has retained length on the hinge pin",
        )
    ctx.allow_overlap(
        bridge,
        right_body,
        elem_a="central_hinge_pin",
        elem_b="right_hinge_sleeve_0",
        reason="The porro body knuckle is represented as a solid sleeve captured on the central hinge pin.",
    )
    ctx.expect_overlap(
        bridge,
        right_body,
        elem_a="central_hinge_pin",
        elem_b="right_hinge_sleeve_0",
        axes="x",
        min_overlap=0.090,
        name="right hinge sleeve has retained length on the hinge pin",
    )

    # Closed objective caps sit just in front of the glass and cover the
    # objective footprint; opening one cap raises it above the barrel.
    ctx.expect_gap(
        left_cap,
        left_body,
        positive_elem="left_cap_disk",
        negative_elem="left_objective_glass",
        axis="x",
        min_gap=0.000,
        max_gap=0.006,
        name="left cap sits just in front of objective glass",
    )
    ctx.expect_overlap(
        left_cap,
        left_body,
        elem_a="left_cap_disk",
        elem_b="left_objective_glass",
        axes="yz",
        min_overlap=0.080,
        name="left cap covers objective glass",
    )
    ctx.expect_gap(
        right_cap,
        right_body,
        positive_elem="right_cap_disk",
        negative_elem="right_objective_glass",
        axis="x",
        min_gap=0.000,
        max_gap=0.006,
        name="right cap sits just in front of objective glass",
    )
    ctx.expect_overlap(
        right_cap,
        right_body,
        elem_a="right_cap_disk",
        elem_b="right_objective_glass",
        axes="yz",
        min_overlap=0.080,
        name="right cap covers objective glass",
    )

    closed_left_cap_aabb = ctx.part_world_aabb(left_cap)
    with ctx.pose({left_cap_joint: 1.65}):
        open_left_cap_aabb = ctx.part_world_aabb(left_cap)
    ctx.check(
        "left objective cap flips upward",
        closed_left_cap_aabb is not None
        and open_left_cap_aabb is not None
        and open_left_cap_aabb[0][2] > closed_left_cap_aabb[0][2] + 0.080,
        details=f"closed={closed_left_cap_aabb}, open={open_left_cap_aabb}",
    )

    closed_right_cap_aabb = ctx.part_world_aabb(right_cap)
    with ctx.pose({right_cap_joint: 1.65}):
        open_right_cap_aabb = ctx.part_world_aabb(right_cap)
    ctx.check(
        "right objective cap flips upward",
        closed_right_cap_aabb is not None
        and open_right_cap_aabb is not None
        and open_right_cap_aabb[0][2] > closed_right_cap_aabb[0][2] + 0.080,
        details=f"closed={closed_right_cap_aabb}, open={open_right_cap_aabb}",
    )

    # The focus wheel is a separate revolute control captured between fork ears.
    ctx.expect_gap(
        focus_wheel,
        bridge,
        positive_elem="wheel_hub",
        negative_elem="focus_yoke_neg",
        axis="y",
        max_gap=0.001,
        max_penetration=0.0,
        name="focus hub touches one yoke cheek",
    )
    ctx.expect_gap(
        bridge,
        focus_wheel,
        positive_elem="focus_yoke_pos",
        negative_elem="wheel_hub",
        axis="y",
        max_gap=0.001,
        max_penetration=0.0,
        name="focus hub touches the other yoke cheek",
    )
    rest_focus_position = ctx.part_world_position(focus_wheel)
    with ctx.pose({focus_joint: 1.2}):
        turned_focus_position = ctx.part_world_position(focus_wheel)
    ctx.check(
        "focus wheel rotates in place",
        rest_focus_position is not None
        and turned_focus_position is not None
        and max(abs(a - b) for a, b in zip(rest_focus_position, turned_focus_position)) < 1e-6,
        details=f"rest={rest_focus_position}, turned={turned_focus_position}",
    )

    # A small central hinge rotation should move the optical half around the
    # fore-aft hinge axis, changing its vertical position while keeping the pin.
    rest_left_aabb = ctx.part_world_aabb(left_body)
    with ctx.pose({left_hinge: 0.12}):
        folded_left_aabb = ctx.part_world_aabb(left_body)
    ctx.check(
        "central hinge has revolute fore-aft motion",
        rest_left_aabb is not None
        and folded_left_aabb is not None
        and folded_left_aabb[1][2] > rest_left_aabb[1][2] + 0.005,
        details=f"rest={rest_left_aabb}, folded={folded_left_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
