from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TireCarcass,
    TireGeometry,
    TireGroove,
    TireShoulder,
    TireSidewall,
    TireTread,
    WheelBore,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def _cylinder_between(
    p0: tuple[float, float, float],
    p1: tuple[float, float, float],
    radius: float,
) -> tuple[Cylinder, Origin]:
    """Return a cylinder visual aligned from p0 to p1 in the containing frame."""
    dx = p1[0] - p0[0]
    dy = p1[1] - p0[1]
    dz = p1[2] - p0[2]
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    if length <= 0.0:
        raise ValueError("cylinder endpoints must be distinct")

    ux, uy, uz = dx / length, dy / length, dz / length
    pitch = math.atan2(math.sqrt(ux * ux + uy * uy), uz)
    yaw = math.atan2(uy, ux) if abs(ux) + abs(uy) > 1e-12 else 0.0
    center = ((p0[0] + p1[0]) * 0.5, (p0[1] + p1[1]) * 0.5, (p0[2] + p1[2]) * 0.5)
    return Cylinder(radius=radius, length=length), Origin(xyz=center, rpy=(0.0, pitch, yaw))


def _rounded_pad_mesh():
    """Soft rounded knee-rest cushion with a shallow crowned top."""
    pad = cq.Workplane("XY").box(0.54, 0.24, 0.064)
    pad = pad.edges("|Z").fillet(0.045)
    pad = pad.edges(">Z").fillet(0.018)
    return mesh_from_cadquery(pad, "rounded_knee_pad", tolerance=0.0015)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="foldable_knee_scooter")

    frame_blue = model.material("powder_coated_blue", rgba=(0.05, 0.17, 0.34, 1.0))
    dark_metal = model.material("dark_anodized_metal", rgba=(0.03, 0.035, 0.04, 1.0))
    polished = model.material("brushed_aluminum", rgba=(0.72, 0.74, 0.72, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.005, 0.005, 0.004, 1.0))
    cushion = model.material("matte_black_cushion", rgba=(0.015, 0.014, 0.013, 1.0))
    grip = model.material("soft_gray_grip", rgba=(0.08, 0.085, 0.085, 1.0))

    tire_mesh = mesh_from_geometry(
        TireGeometry(
            0.080,
            0.046,
            inner_radius=0.055,
            carcass=TireCarcass(belt_width_ratio=0.70, sidewall_bulge=0.04),
            tread=TireTread(style="block", depth=0.004, count=22, land_ratio=0.58),
            grooves=(TireGroove(center_offset=0.0, width=0.0045, depth=0.002),),
            sidewall=TireSidewall(style="rounded", bulge=0.04),
            shoulder=TireShoulder(width=0.006, radius=0.002),
        ),
        "small_scooter_tire",
    )
    rim_mesh = mesh_from_geometry(
        WheelGeometry(
            0.058,
            0.044,
            rim=WheelRim(inner_radius=0.035, flange_height=0.005, flange_thickness=0.003),
            hub=WheelHub(
                radius=0.020,
                width=0.036,
                cap_style="domed",
                bolt_pattern=BoltPattern(count=4, circle_diameter=0.027, hole_diameter=0.0035),
            ),
            face=WheelFace(dish_depth=0.004, front_inset=0.002, rear_inset=0.002),
            spokes=WheelSpokes(style="straight", count=6, thickness=0.0035, window_radius=0.007),
            bore=WheelBore(style="round", diameter=0.022),
        ),
        "small_scooter_rim",
    )
    pad_mesh = _rounded_pad_mesh()

    frame = model.part("frame")

    # Knee platform and the connected metal chassis under it.
    frame.visual(
        pad_mesh,
        origin=Origin(xyz=(-0.16, 0.0, 0.412)),
        material=cushion,
        name="knee_pad",
    )
    frame.visual(
        Box((0.58, 0.19, 0.030)),
        origin=Origin(xyz=(-0.13, 0.0, 0.365)),
        material=dark_metal,
        name="deck_plate",
    )
    frame.visual(
        Box((0.76, 0.060, 0.050)),
        origin=Origin(xyz=(-0.06, 0.0, 0.325)),
        material=frame_blue,
        name="main_spine",
    )

    # Rear fixed supports and cross axle.
    rear_axle, rear_axle_origin = _cylinder_between((-0.47, -0.235, 0.080), (-0.47, 0.235, 0.080), 0.011)
    frame.visual(rear_axle, origin=rear_axle_origin, material=polished, name="rear_axle")
    rear_cross, rear_cross_origin = _cylinder_between((-0.47, -0.150, 0.315), (-0.47, 0.150, 0.315), 0.017)
    frame.visual(rear_cross, origin=rear_cross_origin, material=frame_blue, name="rear_crossbar")
    for side, y in (("left", 0.125), ("right", -0.125)):
        strut, strut_origin = _cylinder_between((-0.47, y, 0.305), (-0.47, y, 0.090), 0.014)
        frame.visual(strut, origin=strut_origin, material=frame_blue, name=f"rear_{side}_support")
        brace, brace_origin = _cylinder_between((-0.36, 0.0, 0.325), (-0.47, y, 0.155), 0.010)
        frame.visual(brace, origin=brace_origin, material=frame_blue, name=f"rear_{side}_brace")

    # Front head tube and folding clamp built into the upright.
    head, head_origin = _cylinder_between((0.365, 0.0, 0.315), (0.440, 0.0, 0.345), 0.022)
    frame.visual(head, origin=head_origin, material=frame_blue, name="front_neck_tube")
    upright, upright_origin = _cylinder_between((0.440, 0.0, 0.245), (0.440, 0.0, 0.625), 0.026)
    frame.visual(upright, origin=upright_origin, material=dark_metal, name="front_upright")
    diagonal, diagonal_origin = _cylinder_between((0.255, 0.0, 0.325), (0.440, 0.0, 0.565), 0.014)
    frame.visual(diagonal, origin=diagonal_origin, material=frame_blue, name="upright_brace")
    frame.visual(
        Box((0.022, 0.016, 0.088)),
        origin=Origin(xyz=(0.440, 0.052, 0.685)),
        material=polished,
        name="clamp_cheek_0",
    )
    frame.visual(
        Box((0.022, 0.016, 0.088)),
        origin=Origin(xyz=(0.440, -0.052, 0.685)),
        material=polished,
        name="clamp_cheek_1",
    )
    hinge_pin_0, hinge_pin_origin_0 = _cylinder_between((0.440, 0.040, 0.685), (0.440, 0.070, 0.685), 0.009)
    frame.visual(hinge_pin_0, origin=hinge_pin_origin_0, material=polished, name="fold_hinge_pin_0")
    hinge_pin_1, hinge_pin_origin_1 = _cylinder_between((0.440, -0.040, 0.685), (0.440, -0.070, 0.685), 0.009)
    frame.visual(hinge_pin_1, origin=hinge_pin_origin_1, material=polished, name="fold_hinge_pin_1")
    frame.visual(
        Box((0.040, 0.110, 0.040)),
        origin=Origin(xyz=(0.440, 0.0, 0.624)),
        material=dark_metal,
        name="folding_clamp_block",
    )

    # Steerable front fork, whose local origin is the vertical steering axis at the lower bearing.
    front_fork = model.part("front_fork")
    fork_crown, fork_crown_origin = _cylinder_between((0.0, -0.150, -0.020), (0.0, 0.150, -0.020), 0.018)
    front_fork.visual(fork_crown, origin=fork_crown_origin, material=polished, name="fork_crown")
    lower_bearing, lower_bearing_origin = _cylinder_between((0.0, 0.0, -0.008), (0.0, 0.0, 0.0), 0.034)
    front_fork.visual(lower_bearing, origin=lower_bearing_origin, material=polished, name="lower_bearing_collar")
    steerer, steerer_origin = _cylinder_between((0.0, 0.0, 0.0), (0.0, 0.0, -0.150), 0.018)
    front_fork.visual(steerer, origin=steerer_origin, material=polished, name="steerer_stub")
    front_axle, front_axle_origin = _cylinder_between((0.0, -0.235, -0.165), (0.0, 0.235, -0.165), 0.011)
    front_fork.visual(front_axle, origin=front_axle_origin, material=polished, name="front_axle")
    for side, y in (("left", 0.125), ("right", -0.125)):
        leg, leg_origin = _cylinder_between((0.0, y, -0.020), (0.0, y, -0.165), 0.013)
        front_fork.visual(leg, origin=leg_origin, material=polished, name=f"fork_{side}_leg")
        dropout, dropout_origin = _cylinder_between((0.0, y, -0.165), (0.0, y + (0.035 if y > 0 else -0.035), -0.165), 0.010)
        front_fork.visual(dropout, origin=dropout_origin, material=polished, name=f"fork_{side}_dropout")

    # Foldable handlebar stem: local origin sits exactly on the transverse hinge pin.
    handlebar_stem = model.part("handlebar_stem")
    barrel, barrel_origin = _cylinder_between((0.0, -0.040, 0.0), (0.0, 0.040, 0.0), 0.027)
    handlebar_stem.visual(barrel, origin=barrel_origin, material=dark_metal, name="hinge_barrel")
    lower_stem, lower_stem_origin = _cylinder_between((0.0, 0.0, 0.020), (0.0, 0.0, 0.575), 0.019)
    handlebar_stem.visual(lower_stem, origin=lower_stem_origin, material=polished, name="folding_stem")
    clamp, clamp_origin = _cylinder_between((0.0, -0.055, 0.575), (0.0, 0.055, 0.575), 0.023)
    handlebar_stem.visual(clamp, origin=clamp_origin, material=dark_metal, name="handlebar_clamp")
    bar, bar_origin = _cylinder_between((0.0, -0.315, 0.610), (0.0, 0.315, 0.610), 0.016)
    handlebar_stem.visual(bar, origin=bar_origin, material=polished, name="handlebar_bar")
    for side, y in (("left", 0.245), ("right", -0.245)):
        grip_bar, grip_origin = _cylinder_between((0.0, y, 0.610), (0.0, y + (0.075 if y > 0 else -0.075), 0.610), 0.020)
        handlebar_stem.visual(grip_bar, origin=grip_origin, material=grip, name=f"{side}_grip")

    def make_wheel(name: str):
        wheel = model.part(name)
        spin_visual_origin = Origin(rpy=(0.0, 0.0, math.pi / 2.0))
        wheel.visual(tire_mesh, origin=spin_visual_origin, material=black_rubber, name="tire")
        wheel.visual(rim_mesh, origin=spin_visual_origin, material=polished, name="rim")
        return wheel

    front_left_wheel = make_wheel("front_left_wheel")
    front_right_wheel = make_wheel("front_right_wheel")
    rear_left_wheel = make_wheel("rear_left_wheel")
    rear_right_wheel = make_wheel("rear_right_wheel")

    model.articulation(
        "steering_axis",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=front_fork,
        origin=Origin(xyz=(0.440, 0.0, 0.245)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=3.0, lower=-0.75, upper=0.75),
    )
    model.articulation(
        "stem_fold_hinge",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=handlebar_stem,
        origin=Origin(xyz=(0.440, 0.0, 0.685)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.5, lower=0.0, upper=1.35),
    )

    wheel_specs = (
        ("front_left_spin", front_fork, front_left_wheel, Origin(xyz=(0.0, 0.190, -0.165))),
        ("front_right_spin", front_fork, front_right_wheel, Origin(xyz=(0.0, -0.190, -0.165))),
        ("rear_left_spin", frame, rear_left_wheel, Origin(xyz=(-0.470, 0.190, 0.080))),
        ("rear_right_spin", frame, rear_right_wheel, Origin(xyz=(-0.470, -0.190, 0.080))),
    )
    for joint_name, parent, child, origin in wheel_specs:
        model.articulation(
            joint_name,
            ArticulationType.CONTINUOUS,
            parent=parent,
            child=child,
            origin=origin,
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=2.0, velocity=20.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    fork = object_model.get_part("front_fork")
    stem = object_model.get_part("handlebar_stem")
    front_left = object_model.get_part("front_left_wheel")
    front_right = object_model.get_part("front_right_wheel")
    rear_left = object_model.get_part("rear_left_wheel")
    rear_right = object_model.get_part("rear_right_wheel")

    steering = object_model.get_articulation("steering_axis")
    fold = object_model.get_articulation("stem_fold_hinge")
    front_spin = object_model.get_articulation("front_left_spin")

    ctx.check(
        "four independently articulated wheels",
        sum(1 for j in object_model.articulations if j.name.endswith("_spin")) == 4,
        details="Expected four continuous wheel spin joints.",
    )
    ctx.check(
        "fork steering and stem folding are revolute",
        steering.articulation_type == ArticulationType.REVOLUTE
        and fold.articulation_type == ArticulationType.REVOLUTE,
        details=f"steering={steering.articulation_type}, fold={fold.articulation_type}",
    )

    ctx.expect_gap(
        frame,
        rear_left,
        axis="z",
        min_gap=0.18,
        positive_elem="deck_plate",
        negative_elem="tire",
        name="deck rides above rear wheel",
    )

    for wheel, support, support_elem in (
        (front_left, fork, "front_axle"),
        (front_right, fork, "front_axle"),
        (rear_left, frame, "rear_axle"),
        (rear_right, frame, "rear_axle"),
    ):
        ctx.allow_overlap(
            support,
            wheel,
            elem_a=support_elem,
            elem_b="rim",
            reason="The axle shaft is intentionally represented as passing through the wheel's captured bearing bore.",
        )
        ctx.expect_overlap(
            wheel,
            support,
            axes="yz",
            elem_a="rim",
            elem_b=support_elem,
            min_overlap=0.015,
            name=f"{wheel.name} retained on axle",
        )

    ctx.expect_overlap(
        front_left,
        fork,
        axes="yz",
        elem_a="rim",
        elem_b="front_axle",
        min_overlap=0.015,
        name="front wheel is carried on fork axle",
    )
    ctx.expect_overlap(
        rear_left,
        frame,
        axes="yz",
        elem_a="rim",
        elem_b="rear_axle",
        min_overlap=0.015,
        name="rear wheel is carried on fixed axle",
    )

    rest_bar_aabb = ctx.part_element_world_aabb(stem, elem="handlebar_bar")
    with ctx.pose({fold: 1.20}):
        folded_bar_aabb = ctx.part_element_world_aabb(stem, elem="handlebar_bar")
    ctx.check(
        "handlebar stem folds rearward",
        rest_bar_aabb is not None
        and folded_bar_aabb is not None
        and folded_bar_aabb[0][0] < rest_bar_aabb[0][0] - 0.45,
        details=f"rest={rest_bar_aabb}, folded={folded_bar_aabb}",
    )

    rest_front_pos = ctx.part_world_position(front_left)
    with ctx.pose({steering: 0.55}):
        steered_front_pos = ctx.part_world_position(front_left)
    ctx.check(
        "front fork steers wheel about vertical axis",
        rest_front_pos is not None
        and steered_front_pos is not None
        and abs(steered_front_pos[0] - rest_front_pos[0]) > 0.030,
        details=f"rest={rest_front_pos}, steered={steered_front_pos}",
    )

    rest_wheel_pos = ctx.part_world_position(front_left)
    with ctx.pose({front_spin: 1.25}):
        spun_wheel_pos = ctx.part_world_position(front_left)
    ctx.check(
        "wheel spin keeps axle center fixed",
        rest_wheel_pos is not None
        and spun_wheel_pos is not None
        and max(abs(a - b) for a, b in zip(rest_wheel_pos, spun_wheel_pos)) < 1e-6,
        details=f"rest={rest_wheel_pos}, spun={spun_wheel_pos}",
    )

    return ctx.report()


object_model = build_object_model()
