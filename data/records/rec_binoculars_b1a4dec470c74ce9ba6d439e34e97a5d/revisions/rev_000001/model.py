from __future__ import annotations

from math import pi

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _cyl_x(radius: float, length: float) -> Cylinder:
    return Cylinder(radius=radius, length=length)


def _origin_x(x: float, y: float, z: float) -> Origin:
    return Origin(xyz=(x, y, z), rpy=(0.0, pi / 2.0, 0.0))


def _origin_y(x: float, y: float, z: float) -> Origin:
    return Origin(xyz=(x, y, z), rpy=(-pi / 2.0, 0.0, 0.0))


def _add_optical_half(
    part,
    *,
    y: float,
    z: float,
    material_body,
    material_black,
    material_glass,
    suffix: str = "",
) -> None:
    """Add one long Porro-binocular optical half around a local X optical axis."""

    part.visual(
        Cylinder(radius=0.125, length=0.86),
        origin=_origin_x(0.05, y, z),
        material=material_body,
        name=f"main_tube{suffix}",
    )
    part.visual(
        Cylinder(radius=0.180, length=0.32),
        origin=_origin_x(0.54, y, z),
        material=material_body,
        name=f"objective_bell{suffix}",
    )
    part.visual(
        Cylinder(radius=0.188, length=0.030),
        origin=_origin_x(0.685, y, z),
        material=material_black,
        name=f"objective_rim{suffix}",
    )
    part.visual(
        Cylinder(radius=0.142, length=0.018),
        origin=_origin_x(0.700, y, z),
        material=material_glass,
        name=f"front_lens{suffix}",
    )
    part.visual(
        Box((0.43, 0.23, 0.22)),
        origin=Origin(xyz=(-0.08, y, z + 0.11)),
        material=material_body,
        name=f"prism_housing{suffix}",
    )
    part.visual(
        Cylinder(radius=0.072, length=0.22),
        origin=_origin_x(-0.48, y, z + 0.02),
        material=material_black,
        name=f"eyepiece_tube{suffix}",
    )
    part.visual(
        Cylinder(radius=0.084, length=0.065),
        origin=_origin_x(-0.620, y, z + 0.02),
        material=material_black,
        name=f"rubber_eyecup{suffix}",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="giant_fork_mounted_porro_binocular")

    matte_black = model.material("matte_black", rgba=(0.006, 0.007, 0.006, 1.0))
    dark_green = model.material("armored_dark_green", rgba=(0.05, 0.09, 0.065, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.015, 0.014, 0.013, 1.0))
    gunmetal = model.material("gunmetal", rgba=(0.25, 0.25, 0.24, 1.0))
    steel = model.material("brushed_steel", rgba=(0.55, 0.56, 0.55, 1.0))
    glass = model.material("coated_blue_glass", rgba=(0.05, 0.16, 0.28, 0.62))

    pedestal = model.part("pedestal")
    pedestal.visual(
        Cylinder(radius=0.38, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=gunmetal,
        name="floor_disk",
    )
    pedestal.visual(
        Cylinder(radius=0.075, length=0.66),
        origin=Origin(xyz=(0.0, 0.0, 0.360)),
        material=gunmetal,
        name="support_column",
    )
    pedestal.visual(
        Cylinder(radius=0.145, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, 0.695)),
        material=steel,
        name="pan_bearing",
    )

    pan_arm = model.part("pan_arm")
    pan_arm.visual(
        Cylinder(radius=0.158, length=0.052),
        origin=Origin(xyz=(0.0, 0.0, 0.026)),
        material=steel,
        name="turntable",
    )
    pan_arm.visual(
        Cylinder(radius=0.052, length=0.44),
        origin=Origin(xyz=(0.0, 0.0, 0.250)),
        material=gunmetal,
        name="riser_post",
    )
    pan_arm.visual(
        Box((0.18, 1.15, 0.105)),
        origin=Origin(xyz=(0.0, 0.0, 0.485)),
        material=gunmetal,
        name="fork_crossbar",
    )
    pan_arm.visual(
        Box((0.12, 0.085, 0.57)),
        origin=Origin(xyz=(0.0, -0.525, 0.740)),
        material=gunmetal,
        name="fork_arm_0",
    )
    pan_arm.visual(
        Box((0.12, 0.085, 0.57)),
        origin=Origin(xyz=(0.0, 0.525, 0.740)),
        material=gunmetal,
        name="fork_arm_1",
    )
    pan_arm.visual(
        Cylinder(radius=0.122, length=0.090),
        origin=_origin_y(0.0, -0.525, 0.950),
        material=steel,
        name="clutch_disk_0",
    )
    pan_arm.visual(
        Cylinder(radius=0.122, length=0.090),
        origin=_origin_y(0.0, 0.525, 0.950),
        material=steel,
        name="clutch_disk_1",
    )
    pan_arm.visual(
        Cylinder(radius=0.026, length=0.66),
        origin=_origin_x(-0.390, 0.0, 0.500),
        material=gunmetal,
        name="pan_handle_bar",
    )
    pan_arm.visual(
        Cylinder(radius=0.045, length=0.13),
        origin=_origin_x(-0.735, 0.0, 0.500),
        material=black_rubber,
        name="pan_handle_grip",
    )

    barrel_0 = model.part("barrel_0")
    _add_optical_half(
        barrel_0,
        y=-0.240,
        z=0.200,
        material_body=dark_green,
        material_black=black_rubber,
        material_glass=glass,
    )
    # Tilt trunnion and exposed clutch hubs carried by the binocular body.
    barrel_0.visual(
        Cylinder(radius=0.042, length=0.880),
        origin=_origin_y(0.0, 0.0, 0.0),
        material=steel,
        name="tilt_axle",
    )
    barrel_0.visual(
        Cylinder(radius=0.104, length=0.080),
        origin=_origin_y(0.0, -0.440, 0.0),
        material=steel,
        name="trunnion_disk_0",
    )
    barrel_0.visual(
        Cylinder(radius=0.104, length=0.080),
        origin=_origin_y(0.0, 0.440, 0.0),
        material=steel,
        name="trunnion_disk_1",
    )

    # Alternating central hinge knuckles along the binocular's optical axis.
    barrel_0.visual(
        Cylinder(radius=0.022, length=1.08),
        origin=_origin_x(0.025, 0.0, 0.320),
        material=steel,
        name="hinge_pin",
    )
    for idx, x in enumerate((-0.405, 0.040, 0.485)):
        barrel_0.visual(
            Cylinder(radius=0.055, length=0.190),
            origin=_origin_x(x, 0.0, 0.320),
            material=matte_black,
            name=f"hinge_knuckle_base_{idx}",
        )
        barrel_0.visual(
            Box((0.11, 0.14, 0.040)),
            origin=Origin(xyz=(x, -0.105, 0.320)),
            material=dark_green,
            name=f"hinge_leaf_base_{idx}",
        )
    barrel_0.visual(
        Box((0.86, 0.085, 0.055)),
        origin=Origin(xyz=(0.05, -0.175, 0.295)),
        material=dark_green,
        name="lower_bridge",
    )
    barrel_0.visual(
        Box((0.22, 0.14, 0.24)),
        origin=Origin(xyz=(0.0, -0.360, 0.100)),
        material=dark_green,
        name="trunnion_saddle_0",
    )
    barrel_0.visual(
        Box((0.22, 0.08, 0.16)),
        origin=Origin(xyz=(0.0, 0.440, 0.070)),
        material=dark_green,
        name="trunnion_saddle_1",
    )
    barrel_0.visual(
        Box((0.16, 0.88, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=dark_green,
        name="trunnion_bridge",
    )

    # Focus wheel support cheeks and shaft, mounted between the prism housings.
    barrel_0.visual(
        Box((0.070, 0.165, 0.030)),
        origin=Origin(xyz=(-0.330, 0.0, 0.340)),
        material=matte_black,
        name="focus_bridge",
    )
    barrel_0.visual(
        Box((0.045, 0.017, 0.145)),
        origin=Origin(xyz=(-0.330, -0.067, 0.405)),
        material=matte_black,
        name="focus_cheek_0",
    )
    barrel_0.visual(
        Box((0.045, 0.017, 0.145)),
        origin=Origin(xyz=(-0.330, 0.067, 0.405)),
        material=matte_black,
        name="focus_cheek_1",
    )
    barrel_0.visual(
        Cylinder(radius=0.013, length=0.150),
        origin=_origin_y(-0.330, 0.0, 0.450),
        material=steel,
        name="focus_axle",
    )

    barrel_1 = model.part("barrel_1")
    _add_optical_half(
        barrel_1,
        y=0.240,
        z=-0.120,
        material_body=dark_green,
        material_black=black_rubber,
        material_glass=glass,
    )
    barrel_1.visual(
        Cylinder(radius=0.055, length=0.190),
        origin=_origin_x(-0.180, 0.0, 0.0),
        material=matte_black,
        name="hinge_knuckle_0",
    )
    barrel_1.visual(
        Box((0.11, 0.155, 0.044)),
        origin=Origin(xyz=(-0.180, 0.105, 0.0)),
        material=dark_green,
        name="hinge_leaf_0",
    )
    barrel_1.visual(
        Cylinder(radius=0.055, length=0.190),
        origin=_origin_x(0.265, 0.0, 0.0),
        material=matte_black,
        name="hinge_knuckle_1",
    )
    barrel_1.visual(
        Box((0.11, 0.155, 0.044)),
        origin=Origin(xyz=(0.265, 0.105, 0.0)),
        material=dark_green,
        name="hinge_leaf_1",
    )

    focus_wheel = model.part("focus_wheel")
    focus_mesh = mesh_from_geometry(
        KnobGeometry(
            0.150,
            0.074,
            body_style="hourglass",
            base_diameter=0.142,
            top_diameter=0.142,
            edge_radius=0.002,
            grip=KnobGrip(style="ribbed", count=32, depth=0.0025, width=0.0022),
        ),
        "ribbed_center_focus_wheel",
    )
    focus_wheel.visual(
        focus_mesh,
        origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
        material=black_rubber,
        name="focus_wheel_body",
    )

    model.articulation(
        "pan_revolve",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=pan_arm,
        origin=Origin(xyz=(0.0, 0.0, 0.730)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.65, lower=-pi, upper=pi),
    )
    model.articulation(
        "tilt_clutch",
        ArticulationType.REVOLUTE,
        parent=pan_arm,
        child=barrel_0,
        origin=Origin(xyz=(0.0, 0.0, 0.950)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=160.0, velocity=0.45, lower=-0.35, upper=0.95),
    )
    model.articulation(
        "central_hinge",
        ArticulationType.REVOLUTE,
        parent=barrel_0,
        child=barrel_1,
        origin=Origin(xyz=(0.0, 0.0, 0.320)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=0.35, lower=-0.16, upper=0.18),
    )
    model.articulation(
        "focus_revolve",
        ArticulationType.REVOLUTE,
        parent=barrel_0,
        child=focus_wheel,
        origin=Origin(xyz=(-0.330, 0.0, 0.450)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.5, velocity=2.2, lower=-2.6, upper=2.6),
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
    # If overlap QC reports an intersection, classify it first: intentional
    # embeddings or nested fits should get a scoped allowance; unintended
    # collisions should be fixed in geometry, support, mount, or pose.

    barrel_0 = object_model.get_part("barrel_0")
    barrel_1 = object_model.get_part("barrel_1")
    pedestal = object_model.get_part("pedestal")
    pan_arm = object_model.get_part("pan_arm")
    focus_wheel = object_model.get_part("focus_wheel")

    central_hinge = object_model.get_articulation("central_hinge")
    tilt_clutch = object_model.get_articulation("tilt_clutch")
    pan_revolve = object_model.get_articulation("pan_revolve")
    focus_revolve = object_model.get_articulation("focus_revolve")

    ctx.allow_overlap(
        barrel_0,
        barrel_1,
        elem_a="hinge_pin",
        elem_b="hinge_knuckle_0",
        reason="The steel hinge pin is intentionally captured inside the moving Porro-body hinge knuckle.",
    )
    ctx.allow_overlap(
        barrel_0,
        barrel_1,
        elem_a="hinge_pin",
        elem_b="hinge_knuckle_1",
        reason="The steel hinge pin is intentionally captured inside the second moving Porro-body hinge knuckle.",
    )
    ctx.allow_overlap(
        barrel_0,
        focus_wheel,
        elem_a="focus_axle",
        elem_b="focus_wheel_body",
        reason="The focus wheel is represented as a solid ribbed wheel captured on its small axle.",
    )

    ctx.expect_gap(
        pan_arm,
        pedestal,
        axis="z",
        positive_elem="turntable",
        negative_elem="pan_bearing",
        max_gap=0.001,
        max_penetration=0.0,
        name="pan turntable sits on bearing",
    )
    ctx.expect_gap(
        barrel_0,
        pan_arm,
        axis="y",
        positive_elem="trunnion_disk_0",
        negative_elem="clutch_disk_0",
        max_gap=0.001,
        max_penetration=0.0,
        name="negative fork clutch captures trunnion",
    )
    ctx.expect_gap(
        pan_arm,
        barrel_0,
        axis="y",
        positive_elem="clutch_disk_1",
        negative_elem="trunnion_disk_1",
        max_gap=0.001,
        max_penetration=0.0,
        name="positive fork clutch captures trunnion",
    )
    for elem in ("hinge_knuckle_0", "hinge_knuckle_1"):
        ctx.expect_within(
            barrel_0,
            barrel_1,
            axes="yz",
            inner_elem="hinge_pin",
            outer_elem=elem,
            margin=0.002,
            name=f"hinge pin centered in {elem}",
        )
        ctx.expect_overlap(
            barrel_0,
            barrel_1,
            axes="x",
            elem_a="hinge_pin",
            elem_b=elem,
            min_overlap=0.16,
            name=f"hinge pin passes through {elem}",
        )
    ctx.expect_within(
        barrel_0,
        focus_wheel,
        axes="xz",
        inner_elem="focus_axle",
        outer_elem="focus_wheel_body",
        margin=0.003,
        name="focus wheel is centered on axle",
    )
    ctx.expect_overlap(
        barrel_0,
        focus_wheel,
        axes="y",
        elem_a="focus_axle",
        elem_b="focus_wheel_body",
        min_overlap=0.060,
        name="focus axle passes through focus wheel",
    )

    rest_objective = ctx.part_element_world_aabb(barrel_0, elem="objective_bell")
    with ctx.pose({tilt_clutch: 0.55}):
        tilted_objective = ctx.part_element_world_aabb(barrel_0, elem="objective_bell")
    ctx.check(
        "tilt clutch raises objectives",
        rest_objective is not None
        and tilted_objective is not None
        and (tilted_objective[0][2] + tilted_objective[1][2])
        > (rest_objective[0][2] + rest_objective[1][2]) + 0.10,
        details=f"rest={rest_objective}, tilted={tilted_objective}",
    )

    rest_hinged_objective = ctx.part_element_world_aabb(barrel_1, elem="objective_bell")
    with ctx.pose({central_hinge: 0.16}):
        hinged_objective = ctx.part_element_world_aabb(barrel_1, elem="objective_bell")
    ctx.check(
        "central hinge folds one barrel",
        rest_hinged_objective is not None
        and hinged_objective is not None
        and (hinged_objective[0][2] + hinged_objective[1][2])
        > (rest_hinged_objective[0][2] + rest_hinged_objective[1][2]) + 0.035,
        details=f"rest={rest_hinged_objective}, folded={hinged_objective}",
    )

    rest_grip = ctx.part_element_world_aabb(pan_arm, elem="pan_handle_grip")
    with ctx.pose({pan_revolve: pi / 2.0}):
        panned_grip = ctx.part_element_world_aabb(pan_arm, elem="pan_handle_grip")
    ctx.check(
        "pan joint swings handle around vertical axis",
        rest_grip is not None
        and panned_grip is not None
        and panned_grip[1][1] < rest_grip[0][1] - 0.60,
        details=f"rest={rest_grip}, panned={panned_grip}",
    )

    ctx.check(
        "focus wheel has bounded revolute travel",
        focus_revolve.motion_limits is not None
        and focus_revolve.motion_limits.lower < 0.0
        and focus_revolve.motion_limits.upper > 0.0,
        details=f"limits={getattr(focus_revolve, 'motion_limits', None)}",
    )

    return ctx.report()


object_model = build_object_model()
