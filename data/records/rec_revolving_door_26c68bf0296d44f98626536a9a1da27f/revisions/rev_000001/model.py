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
    TestContext,
    TestReport,
)


def _rotate_xy(x: float, y: float, angle: float) -> tuple[float, float]:
    c = math.cos(angle)
    s = math.sin(angle)
    return (x * c - y * s, x * s + y * c)


def _add_rotated_box(
    part,
    *,
    size: tuple[float, float, float],
    local_xyz: tuple[float, float, float],
    yaw: float,
    material,
    name: str,
) -> None:
    px, py = _rotate_xy(local_xyz[0], local_xyz[1], yaw)
    part.visual(
        Box(size),
        origin=Origin(xyz=(px, py, local_xyz[2]), rpy=(0.0, 0.0, yaw)),
        material=material,
        name=name,
    )


def _add_wing(
    rotor,
    *,
    yaw: float,
    prefix: str,
    steel,
    glass,
    wing_inner: float,
    wing_outer: float,
    wing_thickness: float,
    wing_bottom: float,
    wing_height: float,
    frame_depth: float,
    glass_thickness: float,
    push_bar_height: float,
) -> None:
    wing_mid_x = (wing_inner + wing_outer) * 0.5
    inner_stile_x = wing_inner + frame_depth * 0.5
    outer_stile_x = wing_outer - frame_depth * 0.5
    clear_width = wing_outer - wing_inner - 2.0 * frame_depth
    glass_width = clear_width + 0.010
    glass_height = wing_height - 2.0 * frame_depth + 0.010

    _add_rotated_box(
        rotor,
        size=(frame_depth, wing_thickness, wing_height),
        local_xyz=(inner_stile_x, 0.0, wing_bottom + wing_height * 0.5),
        yaw=yaw,
        material=steel,
        name=f"{prefix}_inner_stile",
    )
    _add_rotated_box(
        rotor,
        size=(frame_depth, wing_thickness, wing_height),
        local_xyz=(outer_stile_x, 0.0, wing_bottom + wing_height * 0.5),
        yaw=yaw,
        material=steel,
        name=f"{prefix}_outer_stile",
    )
    _add_rotated_box(
        rotor,
        size=(clear_width, wing_thickness, frame_depth),
        local_xyz=(wing_mid_x, 0.0, wing_bottom + frame_depth * 0.5),
        yaw=yaw,
        material=steel,
        name=f"{prefix}_bottom_rail",
    )
    _add_rotated_box(
        rotor,
        size=(clear_width, wing_thickness, frame_depth),
        local_xyz=(wing_mid_x, 0.0, wing_bottom + wing_height - frame_depth * 0.5),
        yaw=yaw,
        material=steel,
        name=f"{prefix}_top_rail",
    )
    _add_rotated_box(
        rotor,
        size=(glass_width, glass_thickness, glass_height),
        local_xyz=(wing_mid_x, 0.0, wing_bottom + wing_height * 0.5),
        yaw=yaw,
        material=glass,
        name=f"{prefix}_glass",
    )
    _add_rotated_box(
        rotor,
        size=(clear_width + 0.010, wing_thickness, 0.060),
        local_xyz=(wing_mid_x, 0.0, push_bar_height),
        yaw=yaw,
        material=steel,
        name=f"{prefix}_push_bar",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="commercial_revolving_door")

    stainless = model.material("stainless", rgba=(0.73, 0.76, 0.79, 1.0))
    dark_stainless = model.material("dark_stainless", rgba=(0.46, 0.49, 0.52, 1.0))
    glass = model.material("glass", rgba=(0.72, 0.85, 0.93, 0.28))
    threshold = model.material("threshold", rgba=(0.20, 0.20, 0.22, 1.0))

    enclosure = model.part("enclosure")

    drum_radius = 1.62
    canopy_thickness = 0.16
    threshold_thickness = 0.06
    clear_height = 2.38
    overall_height = clear_height + canopy_thickness + threshold_thickness
    canopy_center_z = overall_height - canopy_thickness * 0.5
    mullion_height = overall_height - threshold_thickness - canopy_thickness
    mullion_center_z = threshold_thickness + mullion_height * 0.5

    enclosure.visual(
        Cylinder(radius=drum_radius, length=threshold_thickness),
        origin=Origin(xyz=(0.0, 0.0, threshold_thickness * 0.5)),
        material=threshold,
        name="threshold_disk",
    )
    enclosure.visual(
        Cylinder(radius=drum_radius, length=canopy_thickness),
        origin=Origin(xyz=(0.0, 0.0, canopy_center_z)),
        material=dark_stainless,
        name="canopy_disk",
    )
    enclosure.visual(
        Cylinder(radius=0.055, length=overall_height),
        origin=Origin(xyz=(0.0, 0.0, overall_height * 0.5)),
        material=stainless,
        name="center_post",
    )
    enclosure.visual(
        Cylinder(radius=0.070, length=0.080),
        origin=Origin(xyz=(0.0, 0.0, threshold_thickness + 0.040)),
        material=dark_stainless,
        name="bottom_bearing_housing",
    )
    enclosure.visual(
        Cylinder(radius=0.070, length=0.080),
        origin=Origin(xyz=(0.0, 0.0, overall_height - canopy_thickness - 0.040)),
        material=dark_stainless,
        name="top_bearing_housing",
    )

    for angle_deg, width in (
        (60.0, 0.92),
        (100.0, 0.74),
        (140.0, 0.68),
        (-140.0, 0.68),
        (-100.0, 0.74),
        (-60.0, 0.92),
    ):
        angle = math.radians(angle_deg)
        panel_yaw = angle + math.pi * 0.5
        px = (drum_radius - 0.035) * math.cos(angle)
        py = (drum_radius - 0.035) * math.sin(angle)
        enclosure.visual(
            Box((width, 0.022, clear_height + 0.020)),
            origin=Origin(xyz=(px, py, threshold_thickness + clear_height * 0.5), rpy=(0.0, 0.0, panel_yaw)),
            material=glass,
            name=f"fixed_glass_{int(angle_deg)}",
        )

    for angle_deg in (45.0, 80.0, 120.0, 155.0, -155.0, -120.0, -80.0, -45.0):
        angle = math.radians(angle_deg)
        px = (drum_radius - 0.030) * math.cos(angle)
        py = (drum_radius - 0.030) * math.sin(angle)
        enclosure.visual(
            Box((0.090, 0.110, mullion_height)),
            origin=Origin(xyz=(px, py, mullion_center_z), rpy=(0.0, 0.0, angle)),
            material=stainless,
            name=f"perimeter_mullion_{int(angle_deg)}",
        )

    for name, x_sign in (("front", 1.0), ("rear", -1.0)):
        for side_name, y_sign in (("left", 1.0), ("right", -1.0)):
            enclosure.visual(
                Box((0.100, 0.280, mullion_height)),
                origin=Origin(
                    xyz=(x_sign * 1.50, y_sign * 0.78, mullion_center_z),
                ),
                material=stainless,
                name=f"{name}_{side_name}_jamb",
            )
        enclosure.visual(
            Box((0.100, 1.840, 0.120)),
            origin=Origin(xyz=(x_sign * 1.50, 0.0, overall_height - canopy_thickness - 0.020)),
            material=stainless,
            name=f"{name}_portal_header",
        )

    enclosure.inertial = Inertial.from_geometry(
        Cylinder(radius=drum_radius, length=overall_height),
        mass=1100.0,
        origin=Origin(xyz=(0.0, 0.0, overall_height * 0.5)),
    )

    rotor = model.part("rotor")

    collar_height = 2.24
    collar_center_z = 1.19
    rotor.visual(
        Box((0.030, 0.220, collar_height)),
        origin=Origin(xyz=(0.085, 0.0, collar_center_z)),
        material=dark_stainless,
        name="hub_plate_pos_x",
    )
    rotor.visual(
        Box((0.030, 0.220, collar_height)),
        origin=Origin(xyz=(-0.085, 0.0, collar_center_z)),
        material=dark_stainless,
        name="hub_plate_neg_x",
    )
    rotor.visual(
        Box((0.140, 0.030, collar_height)),
        origin=Origin(xyz=(0.0, 0.085, collar_center_z)),
        material=dark_stainless,
        name="hub_plate_pos_y",
    )
    rotor.visual(
        Box((0.140, 0.030, collar_height)),
        origin=Origin(xyz=(0.0, -0.085, collar_center_z)),
        material=dark_stainless,
        name="hub_plate_neg_y",
    )

    wing_inner = 0.10
    wing_outer = 1.42
    wing_thickness = 0.055
    wing_bottom = 0.070
    wing_height = 2.24
    frame_depth = 0.060
    glass_thickness = 0.014
    push_bar_height = 1.08

    for index, yaw in enumerate((0.0, math.pi * 0.5, math.pi, math.pi * 1.5)):
        _add_wing(
            rotor,
            yaw=yaw,
            prefix=f"wing_{index}",
            steel=stainless,
            glass=glass,
            wing_inner=wing_inner,
            wing_outer=wing_outer,
            wing_thickness=wing_thickness,
            wing_bottom=wing_bottom,
            wing_height=wing_height,
            frame_depth=frame_depth,
            glass_thickness=glass_thickness,
            push_bar_height=push_bar_height,
        )

    rotor.inertial = Inertial.from_geometry(
        Cylinder(radius=1.42, length=2.24),
        mass=260.0,
        origin=Origin(xyz=(0.0, 0.0, 1.19)),
    )

    model.articulation(
        "wing_rotation",
        ArticulationType.CONTINUOUS,
        parent=enclosure,
        child=rotor,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=450.0, velocity=1.4),
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
    enclosure = object_model.get_part("enclosure")
    rotor = object_model.get_part("rotor")
    spin = object_model.get_articulation("wing_rotation")

    ctx.check(
        "wing rotation articulation is continuous about vertical axis",
        spin.articulation_type == ArticulationType.CONTINUOUS
        and tuple(round(v, 6) for v in spin.axis) == (0.0, 0.0, 1.0)
        and spin.motion_limits is not None
        and spin.motion_limits.lower is None
        and spin.motion_limits.upper is None,
        details=(
            f"type={spin.articulation_type}, axis={spin.axis}, "
            f"limits={spin.motion_limits}"
        ),
    )
    ctx.expect_origin_distance(
        rotor,
        enclosure,
        axes="xy",
        max_dist=1e-6,
        name="rotor stays centered on the enclosure axis",
    )

    with ctx.pose({spin: 0.0}):
        ctx.expect_gap(
            rotor,
            enclosure,
            axis="z",
            negative_elem="threshold_disk",
            min_gap=0.009,
            max_gap=0.020,
            name="wings clear the threshold",
        )
        ctx.expect_gap(
            enclosure,
            rotor,
            axis="z",
            positive_elem="canopy_disk",
            min_gap=0.110,
            max_gap=0.150,
            name="wings clear the canopy",
        )

    rest_aabb = ctx.part_element_world_aabb(rotor, elem="wing_0_outer_stile")
    with ctx.pose({spin: math.pi / 4.0}):
        ctx.expect_within(
            rotor,
            enclosure,
            axes="xy",
            outer_elem="canopy_disk",
            margin=0.010,
            name="rotor remains within the drum footprint while turning",
        )
        turned_aabb = ctx.part_element_world_aabb(rotor, elem="wing_0_outer_stile")

    def _aabb_center(aabb):
        if aabb is None:
            return None
        lower, upper = aabb
        return tuple((lower[i] + upper[i]) * 0.5 for i in range(3))

    rest_center = _aabb_center(rest_aabb)
    turned_center = _aabb_center(turned_aabb)
    ctx.check(
        "wing assembly actually revolves around the post",
        rest_center is not None
        and turned_center is not None
        and turned_center[0] < rest_center[0] - 0.25
        and turned_center[1] > 0.45,
        details=f"rest_center={rest_center}, turned_center={turned_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
