from __future__ import annotations

import os

_REAL_GETCWD = os.getcwd


def _safe_getcwd() -> str:
    try:
        return _REAL_GETCWD()
    except FileNotFoundError:
        try:
            os.chdir("/")
        except OSError:
            pass
        return "/"


os.getcwd = _safe_getcwd

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    AssetContext,
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)

ASSETS = AssetContext.from_script(__file__)

try:
    os.chdir("/")
except OSError:
    pass


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_cylinder(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _add_member(part, a, b, radius: float, material, name: str | None = None) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _yz_rounded_section(
    *,
    x: float,
    width: float,
    height: float,
    radius: float,
    y_center: float = 0.0,
    z_center: float = 0.0,
) -> list[tuple[float, float, float]]:
    return [
        (x, y_center + y, z_center + z)
        for z, y in rounded_rect_profile(
            height,
            width,
            radius,
            corner_segments=8,
        )
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="roof_cctv_pole", assets=ASSETS)

    roof_gray = model.material("roof_gray", rgba=(0.63, 0.65, 0.67, 1.0))
    membrane_dark = model.material("membrane_dark", rgba=(0.33, 0.35, 0.38, 1.0))
    galvanized = model.material("galvanized", rgba=(0.70, 0.72, 0.74, 1.0))
    dark_coat = model.material("dark_coat", rgba=(0.19, 0.20, 0.22, 1.0))
    camera_body = model.material("camera_body", rgba=(0.88, 0.90, 0.92, 1.0))
    lens_black = model.material("lens_black", rgba=(0.05, 0.05, 0.06, 1.0))
    glass = model.material("glass", rgba=(0.40, 0.56, 0.70, 0.45))
    cable = model.material("cable", rgba=(0.18, 0.18, 0.19, 1.0))

    roof_base = model.part("roof_base")
    roof_base.visual(
        Box((1.70, 1.40, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, -0.04)),
        material=roof_gray,
        name="roof_slab",
    )
    roof_base.visual(
        Box((0.44, 0.44, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=membrane_dark,
        name="mount_pad",
    )
    roof_base.visual(
        Box((0.18, 0.18, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=galvanized,
        name="mount_plate",
    )
    for index, (ax, ay) in enumerate(((-0.055, -0.055), (-0.055, 0.055), (0.055, -0.055), (0.055, 0.055))):
        roof_base.visual(
            Cylinder(radius=0.008, length=0.03),
            origin=Origin(xyz=(ax, ay, 0.015)),
            material=dark_coat,
            name=f"base_bolt_{index}",
        )

    roof_base.visual(
        Cylinder(radius=0.034, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.056)),
        material=galvanized,
        name="mast_sleeve",
    )
    roof_base.visual(
        Cylinder(radius=0.028, length=1.09),
        origin=Origin(xyz=(0.0, 0.0, 0.641)),
        material=galvanized,
        name="mast_lower",
    )
    roof_base.visual(
        Cylinder(radius=0.022, length=0.50),
        origin=Origin(xyz=(0.0, 0.0, 1.436)),
        material=galvanized,
        name="mast_upper",
    )
    roof_base.visual(
        Cylinder(radius=0.050, length=0.034),
        origin=Origin(xyz=(0.0, 0.0, 1.703)),
        material=dark_coat,
        name="mast_cap",
    )

    anchor_specs = {
        "ne": (0.50, 0.40),
        "se": (0.50, -0.40),
        "nw": (-0.50, 0.40),
        "sw": (-0.50, -0.40),
    }
    for name, (ax, ay) in anchor_specs.items():
        roof_base.visual(
            Box((0.09, 0.06, 0.03)),
            origin=Origin(xyz=(ax, ay, 0.015)),
            material=dark_coat,
            name=f"anchor_{name}",
        )
        roof_base.visual(
            Cylinder(radius=0.006, length=0.03),
            origin=Origin(xyz=(ax, ay, 0.03)),
            material=galvanized,
            name=f"anchor_pin_{name}",
        )

    roof_base.visual(
        Box((0.16, 0.11, 0.11)),
        origin=Origin(xyz=(0.18, -0.10, 0.055)),
        material=membrane_dark,
        name="junction_box",
    )
    roof_base.visual(
        Box((0.11, 0.07, 0.055)),
        origin=Origin(xyz=(0.18, -0.10, 0.1375)),
        material=dark_coat,
        name="junction_cap",
    )
    _add_member(
        roof_base,
        (0.105, -0.10, 0.085),
        (0.040, -0.03, 0.132),
        radius=0.012,
        material=galvanized,
        name="conduit_run",
    )

    guy_attach = {
        "ne": (0.024, 0.024, 1.56),
        "se": (0.024, -0.024, 1.56),
        "nw": (-0.024, 0.024, 1.56),
        "sw": (-0.024, -0.024, 1.56),
    }
    for name, (tx, ty, tz) in guy_attach.items():
        roof_base.visual(
            Box((0.024, 0.024, 0.03)),
            origin=Origin(xyz=(tx, ty, 1.555)),
            material=dark_coat,
            name=f"guy_lug_{name}",
        )
        ax, ay = anchor_specs[name]
        _add_member(
            roof_base,
            (ax, ay, 0.03),
            (tx, ty, tz),
            radius=0.003,
            material=cable,
            name=f"guy_wire_{name}",
        )

    roof_base.inertial = Inertial.from_geometry(
        Box((1.70, 1.40, 1.82)),
        mass=42.0,
        origin=Origin(xyz=(0.0, 0.0, 0.87)),
    )

    pan_head = model.part("pan_head")
    pan_head.visual(
        Cylinder(radius=0.044, length=0.06),
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
        material=dark_coat,
        name="pan_base",
    )
    pan_head.visual(
        Cylinder(radius=0.033, length=0.09),
        origin=Origin(xyz=(0.0, 0.0, 0.105)),
        material=galvanized,
        name="pan_motor",
    )
    pan_head.visual(
        Box((0.09, 0.12, 0.09)),
        origin=Origin(xyz=(0.005, 0.0, 0.105)),
        material=dark_coat,
        name="controller_block",
    )
    pan_head.visual(
        Box((0.028, 0.008, 0.03)),
        origin=Origin(xyz=(0.078, -0.064, 0.08)),
        material=dark_coat,
        name="left_yoke_arm",
    )
    pan_head.visual(
        Box((0.05, 0.01, 0.14)),
        origin=Origin(xyz=(0.075, -0.071, 0.083)),
        material=dark_coat,
        name="left_yoke_upright",
    )
    pan_head.visual(
        Box((0.028, 0.008, 0.03)),
        origin=Origin(xyz=(0.078, 0.064, 0.08)),
        material=dark_coat,
        name="right_yoke_arm",
    )
    pan_head.visual(
        Box((0.05, 0.01, 0.14)),
        origin=Origin(xyz=(0.075, 0.071, 0.083)),
        material=dark_coat,
        name="right_yoke_upright",
    )
    pan_head.visual(
        Box((0.08, 0.15, 0.018)),
        origin=Origin(xyz=(0.040, 0.0, 0.154)),
        material=dark_coat,
        name="yoke_top_bridge",
    )
    pan_head.inertial = Inertial.from_geometry(
        Box((0.17, 0.17, 0.18)),
        mass=2.6,
        origin=Origin(xyz=(0.03, 0.0, 0.09)),
    )

    camera_housing = model.part("camera_housing")
    camera_housing.visual(
        Cylinder(radius=0.011, length=0.012),
        origin=Origin(xyz=(0.0, -0.054, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=galvanized,
        name="left_trunnion",
    )
    camera_housing.visual(
        Cylinder(radius=0.011, length=0.012),
        origin=Origin(xyz=(0.0, 0.054, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=galvanized,
        name="right_trunnion",
    )
    camera_housing.visual(
        Box((0.026, 0.016, 0.03)),
        origin=Origin(xyz=(0.023, -0.049, -0.004)),
        material=dark_coat,
        name="left_pivot_cheek",
    )
    camera_housing.visual(
        Box((0.026, 0.016, 0.03)),
        origin=Origin(xyz=(0.023, 0.049, -0.004)),
        material=dark_coat,
        name="right_pivot_cheek",
    )
    camera_housing.visual(
        Box((0.04, 0.10, 0.072)),
        origin=Origin(xyz=(0.056, 0.0, -0.004)),
        material=dark_coat,
        name="rear_pack",
    )
    housing_shell_mesh = mesh_from_geometry(
        section_loft(
            [
                _yz_rounded_section(
                    x=0.072,
                    width=0.076,
                    height=0.070,
                    radius=0.015,
                    z_center=-0.006,
                ),
                _yz_rounded_section(
                    x=0.145,
                    width=0.092,
                    height=0.084,
                    radius=0.019,
                    z_center=-0.006,
                ),
                _yz_rounded_section(
                    x=0.222,
                    width=0.084,
                    height=0.078,
                    radius=0.017,
                    z_center=-0.006,
                ),
            ]
        ),
        ASSETS.mesh_path("camera_housing_shell.obj"),
    )
    camera_housing.visual(
        housing_shell_mesh,
        material=camera_body,
        name="housing_shell",
    )
    front_nose_mesh = mesh_from_geometry(
        section_loft(
            [
                _yz_rounded_section(
                    x=0.216,
                    width=0.078,
                    height=0.064,
                    radius=0.016,
                    z_center=-0.006,
                ),
                _yz_rounded_section(
                    x=0.270,
                    width=0.062,
                    height=0.054,
                    radius=0.014,
                    z_center=-0.006,
                ),
                _yz_rounded_section(
                    x=0.314,
                    width=0.050,
                    height=0.046,
                    radius=0.012,
                    z_center=-0.006,
                ),
            ]
        ),
        ASSETS.mesh_path("camera_front_nose.obj"),
    )
    camera_housing.visual(
        front_nose_mesh,
        material=camera_body,
        name="front_nose",
    )
    camera_housing.visual(
        Box((0.06, 0.10, 0.018)),
        origin=Origin(xyz=(0.225, 0.0, 0.044)),
        material=dark_coat,
        name="sunshade",
    )
    camera_housing.visual(
        Cylinder(radius=0.026, length=0.05),
        origin=Origin(xyz=(0.276, 0.0, -0.006), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=lens_black,
        name="lens_barrel",
    )
    camera_housing.visual(
        Cylinder(radius=0.023, length=0.008),
        origin=Origin(xyz=(0.303, 0.0, -0.006), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=glass,
        name="lens_window",
    )
    camera_housing.inertial = Inertial.from_geometry(
        Box((0.33, 0.12, 0.10)),
        mass=1.8,
        origin=Origin(xyz=(0.16, 0.0, -0.005)),
    )

    model.articulation(
        "mast_pan",
        ArticulationType.REVOLUTE,
        parent=roof_base,
        child=pan_head,
        origin=Origin(xyz=(0.0, 0.0, 1.72)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=2.0,
            lower=-math.pi,
            upper=math.pi,
        ),
    )
    model.articulation(
        "camera_tilt",
        ArticulationType.REVOLUTE,
        parent=pan_head,
        child=camera_housing,
        origin=Origin(xyz=(0.078, 0.0, 0.08)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.4,
            lower=-1.0,
            upper=0.55,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, seed=0)
    roof_base = object_model.get_part("roof_base")
    pan_head = object_model.get_part("pan_head")
    camera_housing = object_model.get_part("camera_housing")
    mast_pan = object_model.get_articulation("mast_pan")
    camera_tilt = object_model.get_articulation("camera_tilt")

    roof_slab = roof_base.get_visual("roof_slab")
    mast_cap = roof_base.get_visual("mast_cap")
    mast_upper = roof_base.get_visual("mast_upper")
    anchor_ne = roof_base.get_visual("anchor_ne")
    anchor_sw = roof_base.get_visual("anchor_sw")
    guy_lug_ne = roof_base.get_visual("guy_lug_ne")
    guy_lug_sw = roof_base.get_visual("guy_lug_sw")
    guy_wire_ne = roof_base.get_visual("guy_wire_ne")
    guy_wire_sw = roof_base.get_visual("guy_wire_sw")

    pan_base = pan_head.get_visual("pan_base")
    controller_block = pan_head.get_visual("controller_block")
    left_yoke_arm = pan_head.get_visual("left_yoke_arm")
    right_yoke_arm = pan_head.get_visual("right_yoke_arm")

    housing_shell = camera_housing.get_visual("housing_shell")
    lens_barrel = camera_housing.get_visual("lens_barrel")
    left_trunnion = camera_housing.get_visual("left_trunnion")
    right_trunnion = camera_housing.get_visual("right_trunnion")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "mast_pan_axis_vertical",
        mast_pan.axis == (0.0, 0.0, 1.0),
        details=f"Expected vertical pan axis, got {mast_pan.axis!r}.",
    )
    ctx.check(
        "camera_tilt_axis_horizontal",
        camera_tilt.axis == (0.0, 1.0, 0.0),
        details=f"Expected horizontal tilt axis along local +Y, got {camera_tilt.axis!r}.",
    )

    pan_limits = mast_pan.motion_limits
    tilt_limits = camera_tilt.motion_limits
    ctx.check(
        "mast_pan_limits_full_sweep",
        pan_limits is not None
        and pan_limits.lower is not None
        and pan_limits.upper is not None
        and math.isclose(pan_limits.lower, -math.pi, abs_tol=1e-6)
        and math.isclose(pan_limits.upper, math.pi, abs_tol=1e-6),
        details="Pan head should sweep a full 360-degree bounded turn.",
    )
    ctx.check(
        "camera_tilt_limits_ptz_range",
        tilt_limits is not None
        and tilt_limits.lower is not None
        and tilt_limits.upper is not None
        and tilt_limits.lower <= -0.95
        and tilt_limits.upper >= 0.50,
        details="Tilt head should look well below the horizon and somewhat above it.",
    )

    ctx.expect_contact(
        pan_head,
        roof_base,
        elem_a=pan_base,
        elem_b=mast_cap,
        name="pan_head_seated_on_mast_cap",
    )
    ctx.expect_overlap(
        pan_head,
        roof_base,
        axes="xy",
        min_overlap=0.08,
        elem_a=pan_base,
        elem_b=mast_cap,
        name="pan_head_centered_over_mast_cap",
    )
    ctx.expect_contact(
        camera_housing,
        pan_head,
        elem_a=left_trunnion,
        elem_b=left_yoke_arm,
        name="left_trunnion_supported_by_yoke",
    )
    ctx.expect_contact(
        camera_housing,
        pan_head,
        elem_a=right_trunnion,
        elem_b=right_yoke_arm,
        name="right_trunnion_supported_by_yoke",
    )
    ctx.expect_gap(
        camera_housing,
        pan_head,
        axis="x",
        min_gap=0.12,
        positive_elem=lens_barrel,
        negative_elem=controller_block,
        name="lens_projects_forward_of_pan_drive",
    )
    ctx.expect_gap(
        camera_housing,
        roof_base,
        axis="z",
        min_gap=1.60,
        positive_elem=housing_shell,
        negative_elem=roof_slab,
        name="camera_housing_above_roof_surface",
    )
    ctx.expect_contact(
        roof_base,
        roof_base,
        elem_a=guy_wire_ne,
        elem_b=anchor_ne,
        name="northeast_guy_wire_hits_anchor",
    )
    ctx.expect_contact(
        roof_base,
        roof_base,
        elem_a=guy_wire_ne,
        elem_b=guy_lug_ne,
        name="northeast_guy_wire_hits_mast_lug",
    )
    ctx.expect_contact(
        roof_base,
        roof_base,
        elem_a=guy_wire_sw,
        elem_b=anchor_sw,
        name="southwest_guy_wire_hits_anchor",
    )
    ctx.expect_contact(
        roof_base,
        roof_base,
        elem_a=guy_wire_sw,
        elem_b=guy_lug_sw,
        name="southwest_guy_wire_hits_mast_lug",
    )

    if pan_limits is not None and pan_limits.lower is not None and pan_limits.upper is not None:
        with ctx.pose({mast_pan: pan_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="mast_pan_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="mast_pan_lower_no_floating")
            ctx.expect_contact(
                pan_head,
                roof_base,
                elem_a=pan_base,
                elem_b=mast_cap,
                name="mast_pan_lower_mount_contact",
            )
        with ctx.pose({mast_pan: pan_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="mast_pan_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="mast_pan_upper_no_floating")
            ctx.expect_contact(
                pan_head,
                roof_base,
                elem_a=pan_base,
                elem_b=mast_cap,
                name="mast_pan_upper_mount_contact",
            )

    if tilt_limits is not None and tilt_limits.lower is not None and tilt_limits.upper is not None:
        with ctx.pose({mast_pan: 1.8, camera_tilt: tilt_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="camera_tilt_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="camera_tilt_lower_no_floating")
            ctx.expect_contact(
                camera_housing,
                pan_head,
                elem_a=left_trunnion,
                elem_b=left_yoke_arm,
                name="camera_tilt_lower_left_support",
            )
            ctx.expect_contact(
                camera_housing,
                pan_head,
                elem_a=right_trunnion,
                elem_b=right_yoke_arm,
                name="camera_tilt_lower_right_support",
            )
            ctx.expect_gap(
                camera_housing,
                roof_base,
                axis="z",
                min_gap=1.60,
                positive_elem=housing_shell,
                negative_elem=roof_slab,
                name="camera_tilt_lower_roof_clearance",
            )
        with ctx.pose({mast_pan: 1.8, camera_tilt: tilt_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="camera_tilt_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="camera_tilt_upper_no_floating")
            ctx.expect_contact(
                camera_housing,
                pan_head,
                elem_a=left_trunnion,
                elem_b=left_yoke_arm,
                name="camera_tilt_upper_left_support",
            )
            ctx.expect_contact(
                camera_housing,
                pan_head,
                elem_a=right_trunnion,
                elem_b=right_yoke_arm,
                name="camera_tilt_upper_right_support",
            )
            ctx.expect_gap(
                camera_housing,
                roof_base,
                axis="z",
                min_gap=1.60,
                positive_elem=housing_shell,
                negative_elem=roof_slab,
                name="camera_tilt_upper_roof_clearance",
            )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
