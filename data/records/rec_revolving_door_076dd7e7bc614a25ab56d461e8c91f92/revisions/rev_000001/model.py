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


def _polar_xy(radius: float, angle: float) -> tuple[float, float]:
    return (radius * math.cos(angle), radius * math.sin(angle))


def _add_tangent_panel(
    part,
    *,
    radius: float,
    angle: float,
    width: float,
    thickness: float,
    height: float,
    z_center: float,
    material,
    name: str,
) -> None:
    x, y = _polar_xy(radius, angle)
    part.visual(
        Box((width, thickness, height)),
        origin=Origin(xyz=(x, y, z_center), rpy=(0.0, 0.0, angle + math.pi / 2.0)),
        material=material,
        name=name,
    )


def _add_rotated_box(
    part,
    *,
    size: tuple[float, float, float],
    xyz: tuple[float, float, float],
    yaw: float,
    material,
    name: str,
) -> None:
    part.visual(
        Box(size),
        origin=Origin(xyz=xyz, rpy=(0.0, 0.0, yaw)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hotel_revolving_door")

    bronze = model.material("bronze", rgba=(0.49, 0.36, 0.24, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.22, 0.20, 0.18, 1.0))
    clear_glass = model.material("clear_glass", rgba=(0.78, 0.88, 0.94, 0.28))

    threshold_thickness = 0.05
    threshold_top = threshold_thickness
    canopy_soffit_thickness = 0.06
    canopy_soffit_bottom = 2.80
    canopy_soffit_top = canopy_soffit_bottom + canopy_soffit_thickness
    canopy_crown_thickness = 0.16
    canopy_crown_bottom = canopy_soffit_top

    sidewall_height = canopy_soffit_bottom - threshold_top
    sidewall_z_center = threshold_top + sidewall_height / 2.0

    rotor_bottom = 0.06
    rotor_top = 2.74
    rotor_height = rotor_top - rotor_bottom
    rotor_z_center = rotor_bottom + rotor_height / 2.0

    frame = model.part("drum_frame")
    frame.visual(
        Cylinder(radius=1.60, length=threshold_thickness),
        origin=Origin(xyz=(0.0, 0.0, threshold_thickness / 2.0)),
        material=dark_metal,
        name="threshold",
    )
    frame.visual(
        Cylinder(radius=1.58, length=canopy_soffit_thickness),
        origin=Origin(
            xyz=(0.0, 0.0, canopy_soffit_bottom + canopy_soffit_thickness / 2.0)
        ),
        material=bronze,
        name="canopy_soffit",
    )
    frame.visual(
        Cylinder(radius=1.74, length=canopy_crown_thickness),
        origin=Origin(
            xyz=(0.0, 0.0, canopy_crown_bottom + canopy_crown_thickness / 2.0)
        ),
        material=bronze,
        name="canopy_crown",
    )

    side_panel_angles_deg = (-55.0, -41.25, -27.5, -13.75, 0.0, 13.75, 27.5, 41.25, 55.0)
    panel_radius = 1.49
    for index, angle_deg in enumerate(side_panel_angles_deg):
        angle = math.radians(angle_deg)
        _add_tangent_panel(
            frame,
            radius=panel_radius,
            angle=angle,
            width=0.36,
            thickness=0.03,
            height=sidewall_height,
            z_center=sidewall_z_center,
            material=clear_glass,
            name=f"side_glass_pos_{index}",
        )
        _add_tangent_panel(
            frame,
            radius=panel_radius,
            angle=angle + math.pi,
            width=0.36,
            thickness=0.03,
            height=sidewall_height,
            z_center=sidewall_z_center,
            material=clear_glass,
            name=f"side_glass_neg_{index}",
        )

    for index, angle_deg in enumerate((68.0, 112.0, 248.0, 292.0)):
        _add_tangent_panel(
            frame,
            radius=1.50,
            angle=math.radians(angle_deg),
            width=0.12,
            thickness=0.14,
            height=sidewall_height,
            z_center=sidewall_z_center,
            material=bronze,
            name=f"opening_post_{index}",
        )

    frame.inertial = Inertial.from_geometry(
        Box((3.48, 3.48, canopy_crown_bottom + canopy_crown_thickness)),
        mass=650.0,
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                (canopy_crown_bottom + canopy_crown_thickness) / 2.0,
            )
        ),
    )

    rotor = model.part("rotor")
    rotor.visual(
        Cylinder(radius=0.09, length=rotor_height),
        origin=Origin(xyz=(0.0, 0.0, rotor_z_center)),
        material=bronze,
        name="central_post",
    )
    rotor.visual(
        Cylinder(radius=0.15, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, threshold_top + 0.05)),
        material=bronze,
        name="bottom_hub",
    )
    rotor.visual(
        Cylinder(radius=0.15, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, rotor_top - 0.05)),
        material=bronze,
        name="top_hub",
    )

    wing_inner_x = 0.10
    wing_outer_x = 1.30
    wing_mid_x = (wing_inner_x + wing_outer_x) / 2.0
    wing_frame_thickness = 0.08
    rail_height = 0.10
    wing_glass_thickness = 0.018
    wing_full_height = rotor_height
    wing_rail_length = 1.25
    wing_glass_length = 1.18
    wing_glass_height = 2.56

    for index in range(4):
        yaw = index * math.pi / 2.0
        inner_xy = _polar_xy(wing_inner_x, yaw)
        outer_xy = _polar_xy(wing_outer_x, yaw)
        mid_xy = _polar_xy(wing_mid_x, yaw)
        _add_rotated_box(
            rotor,
            size=(0.10, wing_frame_thickness, wing_full_height),
            xyz=(inner_xy[0], inner_xy[1], rotor_z_center),
            yaw=yaw,
            material=bronze,
            name=f"wing_inner_stile_{index}",
        )
        _add_rotated_box(
            rotor,
            size=(0.08, wing_frame_thickness, wing_full_height),
            xyz=(outer_xy[0], outer_xy[1], rotor_z_center),
            yaw=yaw,
            material=bronze,
            name=f"wing_outer_stile_{index}",
        )
        _add_rotated_box(
            rotor,
            size=(wing_rail_length, wing_frame_thickness, rail_height),
            xyz=(mid_xy[0], mid_xy[1], rotor_bottom + rail_height / 2.0),
            yaw=yaw,
            material=bronze,
            name=f"wing_bottom_rail_{index}",
        )
        _add_rotated_box(
            rotor,
            size=(wing_rail_length, wing_frame_thickness, rail_height),
            xyz=(mid_xy[0], mid_xy[1], rotor_top - rail_height / 2.0),
            yaw=yaw,
            material=bronze,
            name=f"wing_top_rail_{index}",
        )
        _add_rotated_box(
            rotor,
            size=(wing_glass_length, wing_glass_thickness, wing_glass_height),
            xyz=(mid_xy[0], mid_xy[1], rotor_z_center),
            yaw=yaw,
            material=clear_glass,
            name=f"wing_glass_{index}",
        )

    rotor.inertial = Inertial.from_geometry(
        Box((2.70, 2.70, rotor_height)),
        mass=220.0,
        origin=Origin(xyz=(0.0, 0.0, rotor_z_center)),
    )

    model.articulation(
        "frame_to_rotor",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=rotor,
        origin=Origin(),
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

    frame = object_model.get_part("drum_frame")
    rotor = object_model.get_part("rotor")
    spin = object_model.get_articulation("frame_to_rotor")

    def aabb_center(aabb):
        if aabb is None:
            return None
        lo, hi = aabb
        return tuple((lo[i] + hi[i]) * 0.5 for i in range(3))

    ctx.check(
        "continuous rotor spins about vertical axis",
        spin.articulation_type == ArticulationType.CONTINUOUS
        and spin.axis == (0.0, 0.0, 1.0)
        and spin.motion_limits is not None
        and spin.motion_limits.lower is None
        and spin.motion_limits.upper is None,
        details=f"type={spin.articulation_type}, axis={spin.axis}, limits={spin.motion_limits}",
    )

    ctx.expect_origin_distance(
        frame,
        rotor,
        axes="xy",
        max_dist=0.001,
        name="rotor stays centered in cylindrical drum",
    )
    ctx.expect_gap(
        frame,
        rotor,
        axis="z",
        positive_elem="canopy_soffit",
        negative_elem="top_hub",
        min_gap=0.04,
        max_gap=0.08,
        name="rotor clears fixed overhead canopy",
    )
    ctx.expect_contact(
        rotor,
        frame,
        elem_a="bottom_hub",
        elem_b="threshold",
        contact_tol=0.001,
        name="rotor is carried by threshold bearing contact",
    )

    with ctx.pose({spin: math.pi / 4.0}):
        ctx.expect_within(
            rotor,
            frame,
            axes="xy",
            margin=0.02,
            name="wing assembly stays inside drum envelope when turned",
        )

    rest_aabb = ctx.part_element_world_aabb(rotor, elem="wing_outer_stile_0")
    with ctx.pose({spin: math.pi / 2.0}):
        quarter_turn_aabb = ctx.part_element_world_aabb(rotor, elem="wing_outer_stile_0")

    rest_center = aabb_center(rest_aabb)
    quarter_turn_center = aabb_center(quarter_turn_aabb)
    ctx.check(
        "outer wing sweeps a quarter turn around the post",
        rest_center is not None
        and quarter_turn_center is not None
        and rest_center[0] > 1.15
        and abs(rest_center[1]) < 0.08
        and quarter_turn_center[1] > 1.15
        and abs(quarter_turn_center[0]) < 0.08,
        details=f"rest_center={rest_center}, quarter_turn_center={quarter_turn_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
