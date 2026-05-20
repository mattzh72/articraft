from __future__ import annotations

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


MOTOR_CENTER_Z = -1.05
BLADE_COUNT = 4


def _canopy_mesh():
    """Bell-shaped ceiling canopy, revolved about the vertical fan axis."""
    return LatheGeometry(
        [
            (0.000, 0.000),
            (0.155, 0.000),
            (0.180, -0.020),
            (0.165, -0.052),
            (0.105, -0.095),
            (0.045, -0.115),
            (0.000, -0.115),
        ],
        segments=56,
    )


def _blade_panel_mesh():
    """A thin, pitched, slightly tapered metal ceiling-fan blade."""
    profile = [
        (0.340, -0.055),
        (0.540, -0.071),
        (0.760, -0.078),
        (0.895, -0.066),
        (0.938, -0.035),
        (0.952, 0.000),
        (0.938, 0.035),
        (0.895, 0.066),
        (0.760, 0.078),
        (0.540, 0.071),
        (0.340, 0.055),
    ]
    return (
        ExtrudeGeometry(profile, 0.012, center=True)
        .rotate_x(math.radians(7.0))
        .translate(0.0, 0.0, -0.055)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="industrial_ceiling_fan")

    ceiling_white = model.material("ceiling_white", rgba=(0.86, 0.86, 0.82, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.64, 0.68, 0.70, 1.0))
    galvanized = model.material("brushed_blade_metal", rgba=(0.56, 0.60, 0.62, 1.0))
    black_iron = model.material("black_iron", rgba=(0.05, 0.055, 0.06, 1.0))
    dark_motor = model.material("charcoal_motor", rgba=(0.16, 0.17, 0.18, 1.0))

    mount = model.part("ceiling_mount")
    mount.visual(Box((0.42, 0.42, 0.018)), origin=Origin(xyz=(0.0, 0.0, 0.009)), material=ceiling_white, name="ceiling_plate")
    mount.visual(mesh_from_geometry(_canopy_mesh(), "bell_canopy"), material=satin_steel, name="bell_canopy")
    mount.visual(Cylinder(radius=0.022, length=0.710), origin=Origin(xyz=(0.0, 0.0, -0.465)), material=black_iron, name="downrod")
    mount.visual(Cylinder(radius=0.040, length=0.055), origin=Origin(xyz=(0.0, 0.0, -0.125)), material=black_iron, name="upper_collar")
    mount.visual(Cylinder(radius=0.048, length=0.062), origin=Origin(xyz=(0.0, 0.0, -0.790)), material=black_iron, name="lower_collar")
    mount.visual(Box((0.070, 0.300, 0.026)), origin=Origin(xyz=(0.0, 0.0, -0.795)), material=black_iron, name="hanger_bridge")
    for side, y in enumerate((-0.135, 0.135)):
        mount.visual(
            Box((0.040, 0.018, 0.120)),
            origin=Origin(xyz=(0.0, y, -0.865)),
            material=black_iron,
            name=f"hanger_strap_{side}",
        )

    rotor = model.part("rotor")
    rotor.visual(Cylinder(radius=0.162, length=0.205), origin=Origin(), material=dark_motor, name="motor_drum")
    rotor.visual(Cylinder(radius=0.176, length=0.026), origin=Origin(xyz=(0.0, 0.0, 0.113)), material=black_iron, name="top_flange")
    rotor.visual(Cylinder(radius=0.176, length=0.026), origin=Origin(xyz=(0.0, 0.0, -0.113)), material=black_iron, name="bottom_flange")
    rotor.visual(Cylinder(radius=0.095, length=0.055), origin=Origin(xyz=(0.0, 0.0, 0.124)), material=dark_motor, name="neck")
    rotor.visual(Cylinder(radius=0.066, length=0.100), origin=Origin(xyz=(0.0, 0.0, 0.180)), material=black_iron, name="bearing_collar")
    rotor.visual(Cylinder(radius=0.052, length=0.070), origin=Origin(xyz=(0.0, 0.0, -0.155)), material=satin_steel, name="bottom_hub")
    for index in range(BLADE_COUNT):
        theta = index * math.tau / BLADE_COUNT
        rotor.visual(
            Box((0.055, 0.130, 0.026)),
            origin=Origin(xyz=(0.1775 * math.cos(theta), 0.1775 * math.sin(theta), -0.055), rpy=(0.0, 0.0, theta)),
            material=black_iron,
            name=f"blade_lug_{index}",
        )
    rotor.inertial = Inertial.from_geometry(Cylinder(radius=0.18, length=0.28), mass=8.0)

    axle = model.articulation(
        "axle",
        ArticulationType.CONTINUOUS,
        parent=mount,
        child=rotor,
        origin=Origin(xyz=(0.0, 0.0, MOTOR_CENTER_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=30.0, velocity=20.0),
    )
    axle.meta["description"] = "Central vertical axle carrying the rotating drum, brackets, and four blades."

    for index in range(BLADE_COUNT):
        blade = model.part(f"blade_{index}")
        blade.visual(mesh_from_geometry(_blade_panel_mesh(), f"blade_panel_{index}"), material=galvanized, name="blade_panel")
        blade.visual(Box((0.050, 0.112, 0.022)), origin=Origin(xyz=(0.230, 0.0, -0.055)), material=black_iron, name="hub_clamp")
        blade.visual(Box((0.165, 0.018, 0.020)), origin=Origin(xyz=(0.2875, 0.042, -0.055)), material=black_iron, name="bracket_arm_0")
        blade.visual(Box((0.165, 0.018, 0.020)), origin=Origin(xyz=(0.2875, -0.042, -0.055)), material=black_iron, name="bracket_arm_1")
        blade.visual(Box((0.090, 0.138, 0.014)), origin=Origin(xyz=(0.362, 0.0, -0.055)), material=black_iron, name="blade_saddle")
        blade.visual(Box((0.590, 0.017, 0.006)), origin=Origin(xyz=(0.635, 0.0, -0.049), rpy=(math.radians(7.0), 0.0, 0.0)), material=galvanized, name="pressed_rib")
        for bolt_index, (x, y) in enumerate(((0.335, -0.048), (0.335, 0.048), (0.390, -0.048), (0.390, 0.048))):
            blade.visual(
                Cylinder(radius=0.012, length=0.007),
                origin=Origin(xyz=(x, y, -0.046)),
                material=satin_steel,
                name=f"bolt_{bolt_index}",
            )
        blade.inertial = Inertial.from_geometry(Box((0.72, 0.16, 0.02)), mass=1.1, origin=Origin(xyz=(0.64, 0.0, -0.055)))
        model.articulation(
            f"rotor_to_blade_{index}",
            ArticulationType.FIXED,
            parent=rotor,
            child=blade,
            origin=Origin(rpy=(0.0, 0.0, index * math.tau / BLADE_COUNT)),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    axle = object_model.get_articulation("axle")
    rotor = object_model.get_part("rotor")
    mount = object_model.get_part("ceiling_mount")
    blades = [object_model.get_part(f"blade_{index}") for index in range(BLADE_COUNT)]

    ctx.check("four_blades_present", all(blades), "Expected four separate blade assemblies.")
    ctx.check("central_axle_present", axle is not None, "Expected a central rotating axle.")
    if axle is not None:
        ctx.check("axle_is_continuous", axle.articulation_type == ArticulationType.CONTINUOUS, f"type={axle.articulation_type!r}")
        ctx.check("axle_axis_vertical", tuple(round(v, 3) for v in axle.axis) == (0.0, 0.0, 1.0), f"axis={axle.axis!r}")

    if mount is not None and rotor is not None:
        mount_aabb = ctx.part_world_aabb(mount)
        rotor_aabb = ctx.part_world_aabb(rotor)
        if mount_aabb is not None and rotor_aabb is not None:
            mount_mins, mount_maxs = mount_aabb
            rotor_mins, rotor_maxs = rotor_aabb
            ctx.check(
                "long_downrod_above_motor",
                (mount_maxs[2] - mount_mins[2]) > 0.85 and rotor_maxs[2] < -0.80,
                details=f"mount_z={(mount_mins[2], mount_maxs[2])}, rotor_z={(rotor_mins[2], rotor_maxs[2])}",
            )

    first_blade = blades[0]
    if axle is not None and first_blade is not None:
        with ctx.pose({axle: 0.0}):
            rest_aabb = ctx.part_element_world_aabb(first_blade, elem="blade_panel")
        with ctx.pose({axle: math.pi / 2.0}):
            quarter_aabb = ctx.part_element_world_aabb(first_blade, elem="blade_panel")
        if rest_aabb is not None and quarter_aabb is not None:
            rest_center = tuple((rest_aabb[0][i] + rest_aabb[1][i]) * 0.5 for i in range(3))
            quarter_center = tuple((quarter_aabb[0][i] + quarter_aabb[1][i]) * 0.5 for i in range(3))
            ctx.check(
                "blade_orbits_on_axle",
                rest_center[0] > 0.55 and abs(rest_center[1]) < 0.05 and quarter_center[1] > 0.55 and abs(quarter_center[0]) < 0.05,
                details=f"rest_center={rest_center}, quarter_center={quarter_center}",
            )

    return ctx.report()


object_model = build_object_model()
