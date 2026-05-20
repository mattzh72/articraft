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
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _circle_profile(
    radius: float, *, segments: int = 40, reverse: bool = False
) -> list[tuple[float, float]]:
    points = [
        (
            radius * math.cos(2.0 * math.pi * i / segments),
            radius * math.sin(2.0 * math.pi * i / segments),
        )
        for i in range(segments)
    ]
    return list(reversed(points)) if reverse else points


def _annulus_mesh(
    *,
    outer_radius: float,
    inner_radius: float,
    height: float,
    z_center: float,
    name: str,
):
    geom = ExtrudeWithHolesGeometry(
        _circle_profile(outer_radius),
        [_circle_profile(inner_radius, reverse=True)],
        height,
        cap=True,
        center=True,
        closed=True,
    )
    geom.translate(0.0, 0.0, z_center)
    return mesh_from_geometry(geom, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="site_security_tripod_mast")

    powder_black = model.material("powder_black", rgba=(0.16, 0.16, 0.17, 1.0))
    charcoal = model.material("charcoal", rgba=(0.24, 0.25, 0.27, 1.0))
    anodized_aluminum = model.material(
        "anodized_aluminum", rgba=(0.56, 0.58, 0.60, 1.0)
    )
    rubber = model.material("rubber", rgba=(0.10, 0.10, 0.11, 1.0))
    camera_grey = model.material("camera_grey", rgba=(0.31, 0.33, 0.35, 1.0))
    glass = model.material("glass", rgba=(0.18, 0.24, 0.28, 0.45))

    leg_angles = {
        "front": 0.0,
        "left": 2.0 * math.pi / 3.0,
        "right": 4.0 * math.pi / 3.0,
    }
    hub = model.part("hub")
    hub.visual(
        Cylinder(radius=0.080, length=0.140),
        origin=Origin(xyz=(0.0, 0.0, 0.530)),
        material=powder_black,
        name="hub_lower_collar",
    )
    hub.visual(
        Cylinder(radius=0.060, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.585)),
        material=powder_black,
        name="hub_top_cap",
    )
    for clamp_angle in leg_angles.values():
        c = math.cos(clamp_angle)
        s = math.sin(clamp_angle)
        hub.visual(
            Box((0.018, 0.024, 0.180)),
            origin=Origin(
                xyz=(0.070 * c, 0.070 * s, 0.690),
                rpy=(0.0, 0.0, clamp_angle),
            ),
            material=charcoal,
            name=f"mast_clamp_{int(round(clamp_angle * 1000.0))}",
        )

    hinge_radius = 0.098
    bracket_offset = 0.026

    def add_leg_brackets(
        angle: float,
        *,
        left_name: str,
        right_name: str,
        left_rib_name: str,
        right_rib_name: str,
    ) -> None:
        c = math.cos(angle)
        s = math.sin(angle)
        rib_radius = 0.084
        rib_size_x = 0.016
        hub.visual(
            Box((0.020, 0.012, 0.042)),
            origin=Origin(
                xyz=(
                    hinge_radius * c - bracket_offset * s,
                    hinge_radius * s + bracket_offset * c,
                    0.530,
                ),
                rpy=(0.0, 0.0, angle),
            ),
            material=powder_black,
            name=left_name,
        )
        hub.visual(
            Box((0.020, 0.012, 0.042)),
            origin=Origin(
                xyz=(
                    hinge_radius * c + bracket_offset * s,
                    hinge_radius * s - bracket_offset * c,
                    0.530,
                ),
                rpy=(0.0, 0.0, angle),
            ),
            material=powder_black,
            name=right_name,
        )
        hub.visual(
            Box((rib_size_x, 0.008, 0.042)),
            origin=Origin(
                xyz=(
                    rib_radius * c - bracket_offset * s,
                    rib_radius * s + bracket_offset * c,
                    0.530,
                ),
                rpy=(0.0, 0.0, angle),
            ),
            material=powder_black,
            name=left_rib_name,
        )
        hub.visual(
            Box((rib_size_x, 0.008, 0.042)),
            origin=Origin(
                xyz=(
                    rib_radius * c + bracket_offset * s,
                    rib_radius * s - bracket_offset * c,
                    0.530,
                ),
                rpy=(0.0, 0.0, angle),
            ),
            material=powder_black,
            name=right_rib_name,
        )

    add_leg_brackets(
        leg_angles["front"],
        left_name="front_bracket_left",
        right_name="front_bracket_right",
        left_rib_name="front_bracket_left_rib",
        right_rib_name="front_bracket_right_rib",
    )
    add_leg_brackets(
        leg_angles["left"],
        left_name="left_bracket_left",
        right_name="left_bracket_right",
        left_rib_name="left_bracket_left_rib",
        right_rib_name="left_bracket_right_rib",
    )
    add_leg_brackets(
        leg_angles["right"],
        left_name="right_bracket_left",
        right_name="right_bracket_right",
        left_rib_name="right_bracket_left_rib",
        right_rib_name="right_bracket_right_rib",
    )

    hub.inertial = Inertial.from_geometry(
        Box((0.240, 0.240, 0.430)),
        mass=4.2,
        origin=Origin(xyz=(0.0, 0.0, 0.665)),
    )

    leg_length = 0.880
    leg_connector_length = 0.065
    leg_main_tube_length = leg_length - leg_connector_length
    leg_drop_angle = 0.620
    leg_tube_radius = 0.019
    leg_foot_radius = 0.022
    leg_beta = math.pi / 2.0 + leg_drop_angle
    leg_connector_midpoint = (
        0.5 * leg_connector_length * math.cos(leg_drop_angle),
        0.0,
        -0.5 * leg_connector_length * math.sin(leg_drop_angle),
    )
    leg_midpoint = (
        0.5 * (leg_connector_length + leg_length) * math.cos(leg_drop_angle),
        0.0,
        -0.5 * (leg_connector_length + leg_length) * math.sin(leg_drop_angle),
    )
    leg_tip = (
        leg_length * math.cos(leg_drop_angle),
        0.0,
        -leg_length * math.sin(leg_drop_angle),
    )

    for leg_name in leg_angles:
        leg = model.part(f"leg_{leg_name}")
        leg.visual(
            Cylinder(radius=0.015, length=0.040),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=charcoal,
            name="hinge_barrel",
        )
        leg.visual(
            Box((leg_connector_length, 0.014, 0.024)),
            origin=Origin(xyz=leg_connector_midpoint, rpy=(0.0, leg_drop_angle, 0.0)),
            material=charcoal,
            name="leg_root_arm",
        )
        leg.visual(
            Cylinder(radius=leg_tube_radius, length=leg_main_tube_length),
            origin=Origin(xyz=leg_midpoint, rpy=(0.0, leg_beta, 0.0)),
            material=anodized_aluminum,
            name="leg_tube",
        )
        leg.visual(
            Sphere(radius=leg_foot_radius),
            origin=Origin(xyz=leg_tip),
            material=rubber,
            name="foot_pad",
        )
        leg.inertial = Inertial.from_geometry(
            Box((0.760, 0.060, 0.560)),
            mass=1.0,
            origin=Origin(xyz=(0.360, 0.0, -0.210)),
        )

    mast = model.part("mast")
    mast.visual(
        Cylinder(radius=0.050, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.011)),
        material=charcoal,
        name="mast_base_collar",
    )
    mast.visual(
        Cylinder(radius=0.028, length=0.980),
        origin=Origin(xyz=(0.0, 0.0, 0.512)),
        material=anodized_aluminum,
        name="mast_tube",
    )
    mast.visual(
        Cylinder(radius=0.034, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, 0.970)),
        material=charcoal,
        name="mast_top_collar",
    )
    mast.visual(
        Cylinder(radius=0.046, length=0.048),
        origin=Origin(xyz=(0.0, 0.0, 1.016)),
        material=powder_black,
        name="mast_head_plate",
    )
    mast.inertial = Inertial.from_geometry(
        Box((0.110, 0.110, 1.060)),
        mass=2.6,
        origin=Origin(xyz=(0.0, 0.0, 0.520)),
    )

    pan_head = model.part("pan_head")
    pan_head.visual(
        Cylinder(radius=0.060, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=powder_black,
        name="pan_turntable",
    )
    pan_head.visual(
        Box((0.055, 0.100, 0.070)),
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
        material=charcoal,
        name="pan_pedestal",
    )
    pan_head.visual(
        Box((0.030, 0.012, 0.120)),
        origin=Origin(xyz=(0.0, 0.049, 0.100)),
        material=powder_black,
        name="yoke_left",
    )
    pan_head.visual(
        Box((0.030, 0.012, 0.120)),
        origin=Origin(xyz=(0.0, -0.049, 0.100)),
        material=powder_black,
        name="yoke_right",
    )
    pan_head.inertial = Inertial.from_geometry(
        Box((0.140, 0.120, 0.180)),
        mass=0.9,
        origin=Origin(xyz=(0.0, 0.0, 0.090)),
    )

    camera = model.part("camera")
    camera.visual(
        Cylinder(radius=0.009, length=0.086),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=charcoal,
        name="trunnion_pin",
    )
    camera.visual(
        Box((0.050, 0.060, 0.024)),
        origin=Origin(xyz=(0.025, 0.0, 0.0)),
        material=charcoal,
        name="trunnion_mount",
    )
    camera.visual(
        Box((0.150, 0.080, 0.086)),
        origin=Origin(xyz=(0.115, 0.0, 0.0)),
        material=camera_grey,
        name="body_shell",
    )
    camera.visual(
        Box((0.030, 0.070, 0.070)),
        origin=Origin(xyz=(0.045, 0.0, 0.0)),
        material=charcoal,
        name="rear_housing",
    )
    camera.visual(
        Cylinder(radius=0.026, length=0.065),
        origin=Origin(xyz=(0.198, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=charcoal,
        name="lens_housing",
    )
    camera.visual(
        Cylinder(radius=0.018, length=0.012),
        origin=Origin(xyz=(0.228, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=glass,
        name="lens_glass",
    )
    camera.visual(
        Box((0.060, 0.082, 0.012)),
        origin=Origin(xyz=(0.188, 0.0, 0.048)),
        material=charcoal,
        name="sunshield",
    )
    camera.inertial = Inertial.from_geometry(
        Box((0.210, 0.090, 0.110)),
        mass=1.4,
        origin=Origin(xyz=(0.120, 0.0, 0.0)),
    )

    for leg_name, angle in leg_angles.items():
        c = math.cos(angle)
        s = math.sin(angle)
        model.articulation(
            f"hub_to_leg_{leg_name}",
            ArticulationType.REVOLUTE,
            parent=hub,
            child=f"leg_{leg_name}",
            origin=Origin(xyz=(hinge_radius * c, hinge_radius * s, 0.530), rpy=(0.0, 0.0, angle)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=30.0,
                velocity=1.2,
                lower=0.0,
                upper=1.70,
            ),
        )

    model.articulation(
        "hub_to_mast",
        ArticulationType.PRISMATIC,
        parent=hub,
        child=mast,
        origin=Origin(xyz=(0.0, 0.0, 0.600)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=90.0,
            velocity=0.20,
            lower=0.0,
            upper=0.240,
        ),
    )
    model.articulation(
        "mast_to_pan_head",
        ArticulationType.CONTINUOUS,
        parent=mast,
        child=pan_head,
        origin=Origin(xyz=(0.0, 0.0, 1.040)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.2),
    )
    model.articulation(
        "pan_head_to_camera",
        ArticulationType.REVOLUTE,
        parent=pan_head,
        child=camera,
        origin=Origin(xyz=(0.0, 0.0, 0.110)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=1.4,
            lower=-0.70,
            upper=0.95,
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

    hub = object_model.get_part("hub")
    mast = object_model.get_part("mast")
    pan_head = object_model.get_part("pan_head")
    camera = object_model.get_part("camera")
    front_leg = object_model.get_part("leg_front")
    left_leg = object_model.get_part("leg_left")
    right_leg = object_model.get_part("leg_right")

    front_hinge = object_model.get_articulation("hub_to_leg_front")
    left_hinge = object_model.get_articulation("hub_to_leg_left")
    right_hinge = object_model.get_articulation("hub_to_leg_right")
    mast_slide = object_model.get_articulation("hub_to_mast")
    pan_joint = object_model.get_articulation("mast_to_pan_head")
    tilt_joint = object_model.get_articulation("pan_head_to_camera")

    def elem_center(part_obj, elem_name: str):
        aabb = ctx.part_element_world_aabb(part_obj, elem=elem_name)
        if aabb is None:
            return None
        mins, maxs = aabb
        return tuple((mins[i] + maxs[i]) * 0.5 for i in range(3))

    ctx.check(
        "expected tripod mast parts exist",
        all(
            part is not None
            for part in (hub, mast, pan_head, camera, front_leg, left_leg, right_leg)
        ),
        details="One or more prompt-critical parts could not be resolved.",
    )
    ctx.check(
        "pan joint remains continuous",
        pan_joint.articulation_type == ArticulationType.CONTINUOUS
        and pan_joint.motion_limits is not None
        and pan_joint.motion_limits.lower is None
        and pan_joint.motion_limits.upper is None,
        details=f"pan_joint={pan_joint}",
    )

    ctx.expect_contact(
        front_leg,
        hub,
        elem_a="hinge_barrel",
        elem_b="front_bracket_left",
        name="front leg hinge barrel is supported by the hub bracket",
    )
    ctx.expect_contact(
        left_leg,
        hub,
        elem_a="hinge_barrel",
        elem_b="left_bracket_left",
        name="left leg hinge barrel is supported by the hub bracket",
    )
    ctx.expect_contact(
        right_leg,
        hub,
        elem_a="hinge_barrel",
        elem_b="right_bracket_left",
        name="right leg hinge barrel is supported by the hub bracket",
    )

    ctx.expect_contact(
        mast,
        hub,
        elem_a="mast_base_collar",
        elem_b="hub_top_cap",
        name="mast base collar seats on the hub cap at the retracted pose",
    )
    ctx.expect_overlap(
        mast,
        hub,
        axes="xy",
        elem_a="mast_base_collar",
        elem_b="hub_top_cap",
        min_overlap=0.090,
        name="mast base collar stays centered over the hub cap",
    )
    ctx.expect_contact(
        pan_head,
        mast,
        elem_a="pan_turntable",
        elem_b="mast_head_plate",
        name="pan head turntable sits on the mast head plate",
    )
    ctx.expect_contact(
        camera,
        pan_head,
        elem_a="trunnion_pin",
        elem_b="yoke_left",
        name="camera trunnion sits in the tilt yoke",
    )

    mast_rest = ctx.part_world_position(mast)
    lens_rest = elem_center(camera, "lens_housing")
    front_foot_rest = elem_center(front_leg, "foot_pad")

    with ctx.pose({mast_slide: 0.240}):
        ctx.expect_overlap(
            mast,
            hub,
            axes="xy",
            elem_a="mast_tube",
            elem_b="hub_top_cap",
            min_overlap=0.055,
            name="mast remains centered over the hub when fully extended",
        )
        mast_extended = ctx.part_world_position(mast)

    ctx.check(
        "mast extends upward from the hub",
        mast_rest is not None
        and mast_extended is not None
        and mast_extended[2] > mast_rest[2] + 0.20,
        details=f"rest={mast_rest}, extended={mast_extended}",
    )

    with ctx.pose({front_hinge: 1.40}):
        front_foot_folded = elem_center(front_leg, "foot_pad")

    ctx.check(
        "front leg folds upward on its hinge",
        front_foot_rest is not None
        and front_foot_folded is not None
        and front_foot_folded[2] > front_foot_rest[2] + 0.50,
        details=f"rest={front_foot_rest}, folded={front_foot_folded}",
    )

    with ctx.pose({pan_joint: 1.20}):
        lens_panned = elem_center(camera, "lens_housing")

    ctx.check(
        "camera pans around the mast axis",
        lens_rest is not None
        and lens_panned is not None
        and math.hypot(lens_panned[0] - lens_rest[0], lens_panned[1] - lens_rest[1]) > 0.12,
        details=f"rest={lens_rest}, panned={lens_panned}",
    )

    with ctx.pose({tilt_joint: 0.55}):
        lens_tilted = elem_center(camera, "lens_housing")

    ctx.check(
        "positive tilt raises the camera nose",
        lens_rest is not None
        and lens_tilted is not None
        and lens_tilted[2] > lens_rest[2] + 0.05,
        details=f"rest={lens_rest}, tilted={lens_tilted}",
    )

    ctx.check(
        "all three leg hinges share the same deployed convention",
        front_hinge.motion_limits is not None
        and left_hinge.motion_limits is not None
        and right_hinge.motion_limits is not None
        and front_hinge.axis == left_hinge.axis == right_hinge.axis == (0.0, -1.0, 0.0),
        details=(
            f"front={front_hinge.axis}, left={left_hinge.axis}, right={right_hinge.axis}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
