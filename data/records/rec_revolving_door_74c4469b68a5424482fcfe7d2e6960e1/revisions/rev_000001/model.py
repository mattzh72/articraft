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
    Inertial,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
)


OUTER_RADIUS = 1.18
CANOPY_RADIUS = 1.20
DRUM_PANEL_RADIUS = 1.05
FLOOR_THICKNESS = 0.06
CANOPY_BOTTOM = 2.42
CANOPY_THICKNESS = 0.26


def _rotate_xy(yaw: float, x: float, y: float) -> tuple[float, float]:
    c = math.cos(yaw)
    s = math.sin(yaw)
    return (c * x - s * y, s * x + c * y)


def _polar(radius: float, angle: float) -> tuple[float, float]:
    return (radius * math.cos(angle), radius * math.sin(angle))


def _oriented_origin(
    radius: float,
    angle: float,
    z: float,
    *,
    yaw: float,
    local_xyz: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> Origin:
    cx, cy = _polar(radius, angle)
    dx, dy = _rotate_xy(yaw, local_xyz[0], local_xyz[1])
    return Origin(xyz=(cx + dx, cy + dy, z + local_xyz[2]), rpy=(0.0, 0.0, yaw))


def _add_drum_panel(
    part,
    *,
    angle_deg: float,
    prefix: str,
    bronze,
    glass,
) -> None:
    angle = math.radians(angle_deg)
    yaw = angle + math.pi / 2.0

    panel_width = 0.40
    panel_height = 2.375
    panel_depth = 0.050
    stile_width = 0.040
    rail_height = 0.055
    glass_depth = 0.012
    glass_overlap = 0.008
    bottom = 0.050
    center_z = bottom + panel_height * 0.5

    part.visual(
        Box((stile_width, panel_depth, panel_height)),
        origin=_oriented_origin(
            DRUM_PANEL_RADIUS,
            angle,
            center_z,
            yaw=yaw,
            local_xyz=(-(panel_width * 0.5 - stile_width * 0.5), 0.0, 0.0),
        ),
        material=bronze,
        name=f"{prefix}_left_stile",
    )
    part.visual(
        Box((stile_width, panel_depth, panel_height)),
        origin=_oriented_origin(
            DRUM_PANEL_RADIUS,
            angle,
            center_z,
            yaw=yaw,
            local_xyz=((panel_width * 0.5 - stile_width * 0.5), 0.0, 0.0),
        ),
        material=bronze,
        name=f"{prefix}_right_stile",
    )
    part.visual(
        Box((panel_width, panel_depth, rail_height)),
        origin=_oriented_origin(
            DRUM_PANEL_RADIUS,
            angle,
            center_z,
            yaw=yaw,
            local_xyz=(0.0, 0.0, -(panel_height * 0.5 - rail_height * 0.5)),
        ),
        material=bronze,
        name=f"{prefix}_bottom_rail",
    )
    part.visual(
        Box((panel_width, panel_depth, rail_height)),
        origin=_oriented_origin(
            DRUM_PANEL_RADIUS,
            angle,
            center_z,
            yaw=yaw,
            local_xyz=(0.0, 0.0, panel_height * 0.5 - rail_height * 0.5),
        ),
        material=bronze,
        name=f"{prefix}_top_rail",
    )
    part.visual(
        Box(
            (
                panel_width - 2.0 * stile_width + 2.0 * glass_overlap,
                glass_depth,
                panel_height - 2.0 * rail_height + 2.0 * glass_overlap,
            )
        ),
        origin=_oriented_origin(DRUM_PANEL_RADIUS, angle, center_z, yaw=yaw),
        material=glass,
        name=f"{prefix}_glass",
    )


def _add_rotor_wing(
    part,
    *,
    angle_deg: float,
    prefix: str,
    bronze,
    glass,
) -> None:
    angle = math.radians(angle_deg)
    yaw = angle

    wing_depth = 0.050
    stile_depth = 0.048
    outer_stile_width = 0.040
    inner_stile_width = 0.034
    rail_height = 0.050
    glass_depth = 0.012
    glass_overlap = 0.010

    inner_stile_center = 0.170
    outer_stile_center = 0.905
    rail_span = (outer_stile_center + outer_stile_width * 0.5) - (
        inner_stile_center - inner_stile_width * 0.5
    )
    rail_center = (
        (outer_stile_center + outer_stile_width * 0.5)
        + (inner_stile_center - inner_stile_width * 0.5)
    ) * 0.5

    wing_bottom = 0.030
    wing_top = 2.320
    wing_center_z = (wing_bottom + wing_top) * 0.5
    wing_height = wing_top - wing_bottom

    part.visual(
        Box((inner_stile_width, stile_depth, wing_height)),
        origin=Origin(
            xyz=(inner_stile_center * math.cos(angle), inner_stile_center * math.sin(angle), wing_center_z),
            rpy=(0.0, 0.0, yaw),
        ),
        material=bronze,
        name=f"{prefix}_inner_stile",
    )
    part.visual(
        Box((outer_stile_width, stile_depth, wing_height)),
        origin=Origin(
            xyz=(outer_stile_center * math.cos(angle), outer_stile_center * math.sin(angle), wing_center_z),
            rpy=(0.0, 0.0, yaw),
        ),
        material=bronze,
        name=f"{prefix}_outer_stile",
    )

    for rail_name, z in (
        ("bottom_rail", wing_bottom + rail_height * 0.5),
        ("top_rail", wing_top - rail_height * 0.5),
    ):
        part.visual(
            Box((rail_span, wing_depth, rail_height)),
            origin=Origin(
                xyz=(rail_center * math.cos(angle), rail_center * math.sin(angle), z),
                rpy=(0.0, 0.0, yaw),
            ),
            material=bronze,
            name=f"{prefix}_{rail_name}",
        )

    part.visual(
        Box(
            (
                rail_span - inner_stile_width - outer_stile_width + 2.0 * glass_overlap,
                glass_depth,
                wing_height - 2.0 * rail_height + 2.0 * glass_overlap,
            )
        ),
        origin=Origin(
            xyz=(rail_center * math.cos(angle), rail_center * math.sin(angle), wing_center_z),
            rpy=(0.0, 0.0, yaw),
        ),
        material=glass,
        name=f"{prefix}_glass",
    )
    part.visual(
        Box((0.48, 0.030, 0.060)),
        origin=Origin(
            xyz=(0.54 * math.cos(angle), 0.54 * math.sin(angle), 1.080),
            rpy=(0.0, 0.0, yaw),
        ),
        material=bronze,
        name=f"{prefix}_push_bar",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="heritage_revolving_door")

    dark_bronze = model.material("dark_bronze", rgba=(0.45, 0.31, 0.18, 1.0))
    warm_bronze = model.material("warm_bronze", rgba=(0.61, 0.44, 0.24, 1.0))
    aged_bronze = model.material("aged_bronze", rgba=(0.52, 0.37, 0.22, 1.0))
    clear_glass = model.material("clear_glass", rgba=(0.73, 0.85, 0.90, 0.30))
    stone = model.material("stone", rgba=(0.42, 0.42, 0.40, 1.0))

    housing = model.part("door_housing")
    housing.visual(
        Cylinder(radius=OUTER_RADIUS, length=FLOOR_THICKNESS),
        origin=Origin(xyz=(0.0, 0.0, FLOOR_THICKNESS * 0.5)),
        material=stone,
        name="floor_disk",
    )
    housing.visual(
        Cylinder(radius=CANOPY_RADIUS, length=CANOPY_THICKNESS),
        origin=Origin(xyz=(0.0, 0.0, CANOPY_BOTTOM + CANOPY_THICKNESS * 0.5)),
        material=dark_bronze,
        name="canopy_disk",
    )
    housing.visual(
        Cylinder(radius=CANOPY_RADIUS + 0.03, length=0.100),
        origin=Origin(xyz=(0.0, 0.0, CANOPY_BOTTOM + CANOPY_THICKNESS - 0.020)),
        material=aged_bronze,
        name="canopy_crown",
    )
    housing.visual(
        Cylinder(radius=0.115, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, CANOPY_BOTTOM + CANOPY_THICKNESS + 0.025)),
        material=warm_bronze,
        name="finial_base",
    )
    housing.visual(
        Cylinder(radius=0.048, length=0.150),
        origin=Origin(xyz=(0.0, 0.0, CANOPY_BOTTOM + CANOPY_THICKNESS + 0.125)),
        material=warm_bronze,
        name="finial_stem",
    )
    housing.visual(
        Sphere(radius=0.052),
        origin=Origin(xyz=(0.0, 0.0, CANOPY_BOTTOM + CANOPY_THICKNESS + 0.230)),
        material=warm_bronze,
        name="finial_ball",
    )
    housing.visual(
        Cylinder(radius=0.016, length=0.120),
        origin=Origin(xyz=(0.0, 0.0, CANOPY_BOTTOM + CANOPY_THICKNESS + 0.330)),
        material=warm_bronze,
        name="finial_spire",
    )
    housing.visual(
        Sphere(radius=0.022),
        origin=Origin(xyz=(0.0, 0.0, CANOPY_BOTTOM + CANOPY_THICKNESS + 0.400)),
        material=warm_bronze,
        name="finial_tip",
    )

    for index, angle_deg in enumerate((56.0, 78.0, 102.0, 124.0, 236.0, 258.0, 282.0, 304.0)):
        _add_drum_panel(
            housing,
            angle_deg=angle_deg,
            prefix=f"drum_panel_{index}",
            bronze=warm_bronze,
            glass=clear_glass,
        )

    housing.inertial = Inertial.from_geometry(
        Box((2.46, 2.46, 3.08)),
        mass=1400.0,
        origin=Origin(xyz=(0.0, 0.0, 1.54)),
    )

    rotor = model.part("wing_assembly")
    rotor.visual(
        Cylinder(radius=0.078, length=2.300),
        origin=Origin(xyz=(0.0, 0.0, 1.150)),
        material=warm_bronze,
        name="central_post",
    )
    rotor.visual(
        Cylinder(radius=0.160, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=aged_bronze,
        name="bottom_hub",
    )
    rotor.visual(
        Cylinder(radius=0.145, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 2.285)),
        material=aged_bronze,
        name="top_hub",
    )
    rotor.visual(
        Cylinder(radius=0.100, length=0.080),
        origin=Origin(xyz=(0.0, 0.0, 2.215)),
        material=warm_bronze,
        name="upper_collar",
    )

    for index, angle_deg in enumerate((0.0, 90.0, 180.0, 270.0)):
        _add_rotor_wing(
            rotor,
            angle_deg=angle_deg,
            prefix=f"wing_{index}",
            bronze=warm_bronze,
            glass=clear_glass,
        )

    rotor.inertial = Inertial.from_geometry(
        Cylinder(radius=0.92, length=2.34),
        mass=220.0,
        origin=Origin(xyz=(0.0, 0.0, 1.17)),
    )

    model.articulation(
        "housing_to_wings",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=rotor,
        origin=Origin(xyz=(0.0, 0.0, FLOOR_THICKNESS)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=1.5),
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

    housing = object_model.get_part("door_housing")
    rotor = object_model.get_part("wing_assembly")
    spin = object_model.get_articulation("housing_to_wings")

    ctx.check(
        "continuous center-post articulation exists",
        spin.articulation_type == ArticulationType.CONTINUOUS
        and tuple(round(v, 6) for v in spin.axis) == (0.0, 0.0, 1.0),
        details=f"type={spin.articulation_type}, axis={spin.axis}",
    )
    ctx.expect_origin_distance(
        rotor,
        housing,
        axes="xy",
        max_dist=0.001,
        name="wing assembly stays centered in the drum",
    )
    ctx.expect_within(
        rotor,
        housing,
        axes="xy",
        margin=0.0,
        name="rotor footprint stays inside the drum footprint at rest",
    )
    ctx.expect_gap(
        housing,
        rotor,
        axis="z",
        positive_elem="canopy_disk",
        negative_elem="top_hub",
        min_gap=0.040,
        max_gap=0.090,
        name="top hub clears the canopy",
    )

    def _elem_center(part_obj, elem_name: str) -> tuple[float, float, float] | None:
        aabb = ctx.part_element_world_aabb(part_obj, elem=elem_name)
        if aabb is None:
            return None
        low, high = aabb
        return tuple((low[i] + high[i]) * 0.5 for i in range(3))

    rest_center = _elem_center(rotor, "wing_0_outer_stile")
    with ctx.pose({spin: math.pi / 2.0}):
        quarter_turn_center = _elem_center(rotor, "wing_0_outer_stile")
        ctx.expect_within(
            rotor,
            housing,
            axes="xy",
            margin=0.0,
            name="rotor footprint stays inside the drum footprint at quarter turn",
        )

    ctx.check(
        "wing assembly rotates about the central post axis",
        rest_center is not None
        and quarter_turn_center is not None
        and rest_center[0] > 0.82
        and abs(rest_center[1]) < 0.05
        and quarter_turn_center[1] > 0.82
        and abs(quarter_turn_center[0]) < 0.08,
        details=f"rest={rest_center}, quarter_turn={quarter_turn_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
