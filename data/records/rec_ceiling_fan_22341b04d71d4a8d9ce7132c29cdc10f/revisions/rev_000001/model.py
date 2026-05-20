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
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


HOUSING_OUTER_RADIUS = 0.238
HUB_RADIUS = 0.074
BLADE_OPEN_ANGLE = math.radians(84.0)
BLADE_HINGE_RADIUS = 0.083


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _arc_points(
    center_x: float,
    center_y: float,
    radius: float,
    start_angle: float,
    end_angle: float,
    segments: int,
) -> list[tuple[float, float]]:
    return [
        (
            center_x + radius * math.cos(start_angle + (end_angle - start_angle) * i / segments),
            center_y + radius * math.sin(start_angle + (end_angle - start_angle) * i / segments),
        )
        for i in range(segments + 1)
    ]


def _build_blade_mesh():
    root_x = 0.034
    tip_x = 0.174
    root_half_width = 0.033
    tip_half_width = 0.019
    thickness = 0.004
    tip_center_x = tip_x - tip_half_width

    upper = [
        (root_x, root_half_width),
        (root_x + 0.030, root_half_width * 0.95),
        (root_x + 0.078, 0.026),
        (tip_center_x, tip_half_width),
    ]
    tip_arc = _arc_points(
        tip_center_x,
        0.0,
        tip_half_width,
        math.pi / 2.0,
        -math.pi / 2.0,
        segments=8,
    )
    lower = [
        (root_x + 0.078, -0.026),
        (root_x + 0.030, -root_half_width * 0.95),
        (root_x, -root_half_width),
    ]
    profile = upper + tip_arc[1:-1] + lower
    return ExtrudeGeometry.from_z0(profile, thickness).translate(0.0, 0.0, -thickness * 0.5)


def _build_housing_shell_mesh():
    outer_profile = [
        (0.176, 0.010),
        (0.220, 0.018),
        (0.238, 0.040),
        (0.238, 0.080),
        (0.226, 0.104),
        (0.198, 0.116),
    ]
    inner_profile = [
        (0.166, 0.010),
        (0.208, 0.018),
        (0.216, 0.068),
        (0.204, 0.094),
        (0.104, 0.104),
    ]
    return LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=72,
        start_cap="flat",
        end_cap="flat",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="retractable_blade_ceiling_fan")

    painted_metal = model.material("painted_metal", rgba=(0.86, 0.87, 0.89, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.62, 0.65, 0.69, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.19, 0.20, 0.22, 1.0))
    smoky_acrylic = model.material("smoky_acrylic", rgba=(0.53, 0.58, 0.64, 0.56))

    housing_shell_mesh = _save_mesh("fan_housing_shell", _build_housing_shell_mesh())
    blade_mesh = _save_mesh("fan_blade_panel", _build_blade_mesh())

    motor_housing = model.part("motor_housing")
    motor_housing.visual(
        housing_shell_mesh,
        material=painted_metal,
        name="housing_shell",
    )
    motor_housing.visual(
        Cylinder(radius=0.105, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, 0.110)),
        material=dark_metal,
        name="motor_crown",
    )
    motor_housing.visual(
        Cylinder(radius=0.040, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.078)),
        material=satin_steel,
        name="bearing_collar",
    )
    motor_housing.visual(
        Cylinder(radius=0.012, length=0.180),
        origin=Origin(xyz=(0.0, 0.0, 0.2275)),
        material=satin_steel,
        name="downrod",
    )
    motor_housing.visual(
        Cylinder(radius=0.070, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.3425)),
        material=painted_metal,
        name="ceiling_canopy",
    )
    motor_housing.inertial = Inertial.from_geometry(
        Cylinder(radius=0.238, length=0.420),
        mass=9.0,
        origin=Origin(xyz=(0.0, 0.0, 0.160)),
    )

    rotor_hub = model.part("rotor_hub")
    rotor_hub.visual(
        Cylinder(radius=HUB_RADIUS, length=0.044),
        origin=Origin(xyz=(0.0, 0.0, -0.004)),
        material=dark_metal,
        name="hub_body",
    )
    rotor_hub.visual(
        Sphere(radius=0.052),
        origin=Origin(xyz=(0.0, 0.0, -0.030)),
        material=satin_steel,
        name="hub_nose",
    )
    rotor_hub.visual(
        Cylinder(radius=0.026, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=satin_steel,
        name="rotor_spindle_cover",
    )
    rotor_hub.visual(
        Cylinder(radius=0.012, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, 0.032)),
        material=satin_steel,
        name="rotor_shaft",
    )

    for index in range(3):
        phi = index * math.tau / 3.0
        rotor_hub.visual(
            Box((0.0587, 0.020, 0.014)),
            origin=Origin(
                xyz=(0.047, 0.0, -0.004),
                rpy=(0.0, 0.0, phi),
            ),
            material=dark_metal,
            name=f"blade_arm_{index}",
        )
    rotor_hub.inertial = Inertial.from_geometry(
        Cylinder(radius=0.120, length=0.080),
        mass=1.8,
        origin=Origin(xyz=(0.0, 0.0, -0.005)),
    )

    model.articulation(
        "housing_to_rotor",
        ArticulationType.CONTINUOUS,
        parent=motor_housing,
        child=rotor_hub,
        origin=Origin(xyz=(0.0, 0.0, -0.004)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=24.0),
    )

    for index in range(3):
        blade = model.part(f"blade_{index}")
        blade.visual(
            Cylinder(radius=0.006, length=0.024),
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
            material=satin_steel,
            name="hinge_barrel",
        )
        blade.visual(
            Box((0.032, 0.014, 0.010)),
            origin=Origin(xyz=(0.019, 0.0, 0.0)),
            material=dark_metal,
            name="blade_root",
        )
        blade.visual(
            blade_mesh,
            material=smoky_acrylic,
            name="blade_panel",
        )
        blade.visual(
            Box((0.010, 0.018, 0.004)),
            origin=Origin(xyz=(0.169, 0.0, 0.0)),
            material=smoky_acrylic,
            name="blade_tip",
        )
        blade.inertial = Inertial.from_geometry(
            Box((0.176, 0.070, 0.024)),
            mass=0.18,
            origin=Origin(xyz=(0.088, 0.0, 0.0)),
        )

        phi = index * math.tau / 3.0
        model.articulation(
            f"hub_to_blade_{index}",
            ArticulationType.REVOLUTE,
            parent=rotor_hub,
            child=blade,
            origin=Origin(
                xyz=(BLADE_HINGE_RADIUS * math.cos(phi), BLADE_HINGE_RADIUS * math.sin(phi), -0.002),
                rpy=(0.0, 0.0, phi - BLADE_OPEN_ANGLE),
            ),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=1.2,
                velocity=5.0,
                lower=0.0,
                upper=BLADE_OPEN_ANGLE,
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

    housing = object_model.get_part("motor_housing")
    rotor = object_model.get_part("rotor_hub")
    spin = object_model.get_articulation("housing_to_rotor")
    blades = [object_model.get_part(f"blade_{i}") for i in range(3)]
    blade_joints = [object_model.get_articulation(f"hub_to_blade_{i}") for i in range(3)]

    for blade in blades:
        ctx.expect_within(
            blade,
            housing,
            axes="xy",
            margin=0.004,
            name=f"{blade.name} folds within the housing footprint",
        )

    ctx.expect_within(
        rotor,
        housing,
        axes="xy",
        margin=0.010,
        name="rotor hub stays centered within the motor housing",
    )

    def _aabb_center(aabb):
        if aabb is None:
            return None
        (min_x, min_y, min_z), (max_x, max_y, max_z) = aabb
        return (
            (min_x + max_x) * 0.5,
            (min_y + max_y) * 0.5,
            (min_z + max_z) * 0.5,
        )

    folded_tip = _aabb_center(ctx.part_element_world_aabb("blade_0", elem="blade_tip"))
    folded_housing = ctx.part_world_aabb(housing)

    with ctx.pose({joint: BLADE_OPEN_ANGLE for joint in blade_joints}):
        open_tip_aabb = ctx.part_element_world_aabb("blade_0", elem="blade_tip")
        open_tip = _aabb_center(open_tip_aabb)
        housing_aabb = ctx.part_world_aabb(housing)

        ctx.expect_gap(
            housing,
            blades[0],
            axis="z",
            min_gap=0.002,
            positive_elem="housing_shell",
            name="opened blade clears below the housing shell",
        )
        ctx.check(
            "blade_0 swings outward from the housing",
            folded_tip is not None
            and open_tip is not None
            and open_tip[0] > folded_tip[0] + 0.08,
            details=f"folded_tip={folded_tip}, open_tip={open_tip}",
        )
        ctx.check(
            "blade_0 tip extends beyond the housing when deployed",
            open_tip_aabb is not None
            and housing_aabb is not None
            and open_tip_aabb[0][0] > housing_aabb[1][0] + 0.008,
            details=f"open_tip_aabb={open_tip_aabb}, housing_aabb={housing_aabb}",
        )

    with ctx.pose(
        {joint: BLADE_OPEN_ANGLE for joint in blade_joints}
        | {spin: math.pi / 2.0}
    ):
        spun_tip = _aabb_center(ctx.part_element_world_aabb("blade_0", elem="blade_tip"))

    ctx.check(
        "continuous rotor spin carries the opened blade around the vertical axis",
        open_tip is not None
        and spun_tip is not None
        and abs(spun_tip[0]) < 0.070
        and spun_tip[1] > open_tip[0] - 0.060,
        details=f"open_tip={open_tip}, spun_tip={spun_tip}",
    )

    ctx.check(
        "housing footprint remains substantial and round enough for retractable blades",
        folded_housing is not None
        and (folded_housing[1][0] - folded_housing[0][0]) > 0.45
        and (folded_housing[1][1] - folded_housing[0][1]) > 0.45,
        details=f"housing_aabb={folded_housing}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
