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
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _shell_mesh(name: str, outer_profile, inner_profile):
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            outer_profile,
            inner_profile,
            segments=72,
            start_cap="flat",
            end_cap="flat",
            lip_samples=8,
        ),
        name,
    )


def _dovetail_rail_mesh(
    name: str,
    *,
    length: float,
    height: float,
    inner_width: float,
    outer_width: float,
):
    profile = [
        (-height * 0.5, -outer_width * 0.5),
        (height * 0.5, -inner_width * 0.5),
        (height * 0.5, inner_width * 0.5),
        (-height * 0.5, outer_width * 0.5),
    ]
    geom = ExtrudeGeometry(profile, length, center=True)
    geom.rotate_y(math.pi / 2.0)
    return mesh_from_geometry(geom, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="perspective_control_shift_lens")

    anodized_black = model.material("anodized_black", rgba=(0.10, 0.10, 0.11, 1.0))
    satin_black = model.material("satin_black", rgba=(0.16, 0.16, 0.17, 1.0))
    dark_rubber = model.material("dark_rubber", rgba=(0.07, 0.07, 0.08, 1.0))
    mount_steel = model.material("mount_steel", rgba=(0.62, 0.64, 0.67, 1.0))
    optics_glass = model.material("optics_glass", rgba=(0.18, 0.24, 0.30, 0.42))

    mount_base = model.part("mount_base")
    mount_base.visual(
        Cylinder(radius=0.0315, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=mount_steel,
        name="mount_flange",
    )
    mount_base.visual(
        Cylinder(radius=0.0290, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.016)),
        material=satin_black,
        name="bearing_drum",
    )
    for index, angle in enumerate((0.2, 2.28, 4.37)):
        c = math.cos(angle)
        s = math.sin(angle)
        mount_base.visual(
            Box((0.010, 0.004, 0.003)),
            origin=Origin(
                xyz=(0.027 * c, 0.027 * s, 0.003),
                rpy=(0.0, 0.0, angle),
            ),
            material=mount_steel,
            name=f"bayonet_tab_{index}",
        )
    mount_base.visual(
        Box((0.008, 0.004, 0.003)),
        origin=Origin(xyz=(0.018, -0.022, 0.0045), rpy=(0.0, 0.0, -0.55)),
        material=mount_steel,
        name="mount_lock_ramp",
    )
    mount_base.inertial = Inertial.from_geometry(
        Cylinder(radius=0.032, length=0.026),
        mass=0.14,
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
    )

    rotation_ring = model.part("rotation_ring")
    rotation_collar_mesh = _shell_mesh(
        "rotation_collar",
        [
            (0.0390, -0.007),
            (0.0405, -0.003),
            (0.0405, 0.003),
            (0.0390, 0.007),
        ],
        [
            (0.0292, -0.007),
            (0.0292, 0.007),
        ],
    )
    top_rail_mesh = _dovetail_rail_mesh(
        "top_dovetail_rail",
        length=0.088,
        height=0.006,
        inner_width=0.007,
        outer_width=0.011,
    )
    bottom_rail_mesh = _dovetail_rail_mesh(
        "bottom_dovetail_rail",
        length=0.088,
        height=0.006,
        inner_width=0.007,
        outer_width=0.011,
    )
    rotation_ring.visual(
        rotation_collar_mesh,
        material=anodized_black,
        name="rotation_collar",
    )
    rotation_ring.visual(
        Box((0.008, 0.050, 0.010)),
        origin=Origin(xyz=(-0.040, 0.0, 0.008)),
        material=anodized_black,
        name="left_guide_arm",
    )
    rotation_ring.visual(
        Box((0.008, 0.050, 0.010)),
        origin=Origin(xyz=(0.040, 0.0, 0.008)),
        material=anodized_black,
        name="right_guide_arm",
    )
    rotation_ring.visual(
        top_rail_mesh,
        origin=Origin(xyz=(0.0, 0.018, 0.010)),
        material=satin_black,
        name="top_rail",
    )
    rotation_ring.visual(
        bottom_rail_mesh,
        origin=Origin(xyz=(0.0, -0.018, 0.010), rpy=(0.0, 0.0, math.pi)),
        material=satin_black,
        name="bottom_rail",
    )
    rotation_ring.visual(
        Cylinder(radius=0.005, length=0.012),
        origin=Origin(xyz=(0.033, 0.0, -0.001), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_black,
        name="rotation_lock_knob",
    )
    rotation_ring.inertial = Inertial.from_geometry(
        Box((0.096, 0.060, 0.026)),
        mass=0.22,
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
    )

    optical_assembly = model.part("optical_assembly")
    outer_barrel_mesh = _shell_mesh(
        "outer_barrel_shell",
        [
            (0.024, 0.018),
            (0.027, 0.022),
            (0.031, 0.032),
            (0.036, 0.050),
            (0.041, 0.076),
            (0.045, 0.106),
            (0.047, 0.132),
        ],
        [
            (0.020, 0.018),
            (0.022, 0.022),
            (0.026, 0.032),
            (0.031, 0.050),
            (0.035, 0.076),
            (0.039, 0.106),
            (0.040, 0.132),
        ],
    )
    focus_grip_mesh = _shell_mesh(
        "focus_grip_shell",
        [
            (0.0425, 0.060),
            (0.0440, 0.066),
            (0.0440, 0.094),
            (0.0425, 0.100),
        ],
        [
            (0.0385, 0.060),
            (0.0400, 0.066),
            (0.0400, 0.094),
            (0.0385, 0.100),
        ],
    )
    front_trim_mesh = _shell_mesh(
        "front_trim_shell",
        [
            (0.0470, 0.118),
            (0.0485, 0.124),
            (0.0485, 0.136),
            (0.0470, 0.140),
        ],
        [
            (0.0410, 0.118),
            (0.0410, 0.140),
        ],
    )
    optical_assembly.visual(
        Box((0.046, 0.018, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=satin_black,
        name="guide_shoe",
    )
    optical_assembly.visual(
        Box((0.046, 0.004, 0.004)),
        origin=Origin(xyz=(0.0, 0.010, 0.001)),
        material=satin_black,
        name="guide_flange_top",
    )
    optical_assembly.visual(
        Box((0.046, 0.004, 0.004)),
        origin=Origin(xyz=(0.0, -0.010, 0.001)),
        material=satin_black,
        name="guide_flange_bottom",
    )
    optical_assembly.visual(
        Box((0.050, 0.036, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.011)),
        material=anodized_black,
        name="shift_block",
    )
    optical_assembly.visual(
        Cylinder(radius=0.023, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=satin_black,
        name="rear_throat",
    )
    optical_assembly.visual(
        Cylinder(radius=0.0048, length=0.010),
        origin=Origin(xyz=(0.018, 0.021, 0.013), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_black,
        name="shift_lock_knob",
    )
    optical_assembly.visual(
        outer_barrel_mesh,
        material=anodized_black,
        name="outer_barrel_shell",
    )
    optical_assembly.visual(
        focus_grip_mesh,
        material=dark_rubber,
        name="focus_grip",
    )
    optical_assembly.visual(
        front_trim_mesh,
        material=satin_black,
        name="front_trim",
    )
    optical_assembly.visual(
        Cylinder(radius=0.0405, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.124)),
        material=optics_glass,
        name="front_glass",
    )
    optical_assembly.inertial = Inertial.from_geometry(
        Box((0.098, 0.098, 0.144)),
        mass=0.86,
        origin=Origin(xyz=(0.0, 0.0, 0.072)),
    )

    model.articulation(
        "mount_to_rotation_ring",
        ArticulationType.REVOLUTE,
        parent=mount_base,
        child=rotation_ring,
        origin=Origin(xyz=(0.0, 0.0, 0.016)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=1.0,
            lower=-math.pi / 2.0,
            upper=math.pi / 2.0,
        ),
    )
    model.articulation(
        "rotation_ring_to_optics",
        ArticulationType.PRISMATIC,
        parent=rotation_ring,
        child=optical_assembly,
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=0.04,
            lower=-0.012,
            upper=0.012,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    mount_base = object_model.get_part("mount_base")
    rotation_ring = object_model.get_part("rotation_ring")
    optical_assembly = object_model.get_part("optical_assembly")
    ring_rotation = object_model.get_articulation("mount_to_rotation_ring")
    shift_slide = object_model.get_articulation("rotation_ring_to_optics")

    bearing_drum = mount_base.get_visual("bearing_drum")
    rotation_collar = rotation_ring.get_visual("rotation_collar")
    top_rail = rotation_ring.get_visual("top_rail")
    bottom_rail = rotation_ring.get_visual("bottom_rail")
    guide_shoe = optical_assembly.get_visual("guide_shoe")
    shift_block = optical_assembly.get_visual("shift_block")

    with ctx.pose({ring_rotation: 0.0, shift_slide: 0.0}):
        ctx.expect_overlap(
            rotation_ring,
            mount_base,
            axes="xy",
            elem_a=rotation_collar,
            elem_b=bearing_drum,
            min_overlap=0.055,
            name="rotation collar stays concentric on the mount bearing drum",
        )
        ctx.expect_gap(
            optical_assembly,
            mount_base,
            axis="z",
            positive_elem=shift_block,
            negative_elem=bearing_drum,
            min_gap=0.004,
            max_gap=0.018,
            name="shift block sits forward of the mount base",
        )
        ctx.expect_gap(
            rotation_ring,
            optical_assembly,
            axis="y",
            positive_elem=top_rail,
            negative_elem=guide_shoe,
            min_gap=0.001,
            max_gap=0.004,
            name="top dovetail rail clears the sliding shoe",
        )
        ctx.expect_gap(
            optical_assembly,
            rotation_ring,
            axis="y",
            positive_elem=guide_shoe,
            negative_elem=bottom_rail,
            min_gap=0.001,
            max_gap=0.004,
            name="bottom dovetail rail clears the sliding shoe",
        )

    slide_limits = shift_slide.motion_limits
    lower = -0.012 if slide_limits is None or slide_limits.lower is None else slide_limits.lower
    upper = 0.012 if slide_limits is None or slide_limits.upper is None else slide_limits.upper

    for pose_value, label in ((lower, "negative"), (upper, "positive")):
        with ctx.pose({ring_rotation: 0.0, shift_slide: pose_value}):
            ctx.expect_overlap(
                optical_assembly,
                rotation_ring,
                axes="x",
                elem_a=guide_shoe,
                elem_b=top_rail,
                min_overlap=0.014,
                name=f"{label} shift retains insertion under the top rail",
            )
            ctx.expect_overlap(
                optical_assembly,
                rotation_ring,
                axes="x",
                elem_a=guide_shoe,
                elem_b=bottom_rail,
                min_overlap=0.014,
                name=f"{label} shift retains insertion above the bottom rail",
            )

    with ctx.pose({ring_rotation: 0.0, shift_slide: lower}):
        x_lower = ctx.part_world_position(optical_assembly)
    with ctx.pose({ring_rotation: 0.0, shift_slide: upper}):
        x_upper = ctx.part_world_position(optical_assembly)
    ctx.check(
        "shift motion is lateral before the ring is rotated",
        x_lower is not None
        and x_upper is not None
        and x_upper[0] > x_lower[0] + 0.020
        and abs(x_upper[1] - x_lower[1]) < 0.004,
        details=f"lower={x_lower}, upper={x_upper}",
    )

    with ctx.pose({ring_rotation: math.pi / 2.0, shift_slide: lower}):
        y_lower = ctx.part_world_position(optical_assembly)
    with ctx.pose({ring_rotation: math.pi / 2.0, shift_slide: upper}):
        y_upper = ctx.part_world_position(optical_assembly)
    ctx.check(
        "rotation ring redirects the shift travel around the optical axis",
        y_lower is not None
        and y_upper is not None
        and y_upper[1] > y_lower[1] + 0.020
        and abs(y_upper[0] - y_lower[0]) < 0.004,
        details=f"lower={y_lower}, upper={y_upper}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
