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
    TorusGeometry,
    mesh_from_geometry,
)


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> float:
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


def _add_member(part, a, b, radius: float, material, *, name: str | None = None) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _aabb_center(aabb) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    (min_x, min_y, min_z), (max_x, max_y, max_z) = aabb
    return (
        (min_x + max_x) * 0.5,
        (min_y + max_y) * 0.5,
        (min_z + max_z) * 0.5,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="searchlight_tower")

    concrete = model.material("concrete", rgba=(0.66, 0.66, 0.64, 1.0))
    steel = model.material("steel", rgba=(0.46, 0.48, 0.50, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.22, 0.24, 0.27, 1.0))
    painted_gray = model.material("painted_gray", rgba=(0.57, 0.60, 0.63, 1.0))
    housing_black = model.material("housing_black", rgba=(0.11, 0.12, 0.13, 1.0))
    lens_glass = model.material("lens_glass", rgba=(0.72, 0.85, 0.94, 0.38))
    warning_red = model.material("warning_red", rgba=(0.68, 0.14, 0.12, 1.0))

    guard_hoop_mesh = mesh_from_geometry(
        TorusGeometry(radius=0.34, tube=0.012, radial_segments=16, tubular_segments=64),
        "guard_hoop",
    )

    tower_base = model.part("tower_base")
    tower_base.visual(
        Box((1.40, 1.40, 0.18)),
        origin=Origin(xyz=(0.0, 0.0, 0.09)),
        material=concrete,
        name="foundation_pad",
    )
    tower_base.visual(
        Box((0.78, 0.78, 0.12)),
        origin=Origin(xyz=(0.0, 0.0, 0.24)),
        material=dark_steel,
        name="base_plinth",
    )
    tower_base.visual(
        Cylinder(radius=0.12, length=2.00),
        origin=Origin(xyz=(0.0, 0.0, 1.30)),
        material=painted_gray,
        name="mast_column",
    )
    tower_base.visual(
        Cylinder(radius=0.17, length=0.22),
        origin=Origin(xyz=(0.0, 0.0, 2.10)),
        material=dark_steel,
        name="mast_collar",
    )
    tower_base.visual(
        Cylinder(radius=0.19, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 2.26)),
        material=steel,
        name="bearing_cap",
    )
    tower_base.visual(
        Cylinder(radius=0.23, length=0.05),
        origin=Origin(xyz=(0.0, 0.0, 2.22)),
        material=dark_steel,
        name="guard_support_collar",
    )
    tower_base.visual(
        guard_hoop_mesh,
        origin=Origin(xyz=(0.0, 0.0, 2.48)),
        material=warning_red,
        name="guard_hoop",
    )

    for sx in (-0.34, 0.34):
        for sy in (-0.34, 0.34):
            tower_base.visual(
                Box((0.18, 0.18, 0.06)),
                origin=Origin(xyz=(sx, sy, 0.21)),
                material=steel,
                name=f"anchor_block_{'p' if sx > 0 else 'n'}x_{'p' if sy > 0 else 'n'}y",
            )
            tower_base.visual(
                Cylinder(radius=0.015, length=0.09),
                origin=Origin(xyz=(sx, sy, 0.255)),
                material=steel,
                name=f"anchor_bolt_{'p' if sx > 0 else 'n'}x_{'p' if sy > 0 else 'n'}y",
            )

    brace_top_z = 1.05
    for sx in (-0.31, 0.31):
        for sy in (-0.31, 0.31):
            _add_member(
                tower_base,
                (sx, sy, 0.27),
                (0.0, 0.0, brace_top_z),
                0.028,
                painted_gray,
                name=f"mast_brace_{'p' if sx > 0 else 'n'}x_{'p' if sy > 0 else 'n'}y",
            )

    for x, y in ((0.21, 0.0), (-0.21, 0.0), (0.0, 0.21), (0.0, -0.21)):
        ring_x = 0.34 if x > 0 else (-0.34 if x < 0 else 0.0)
        ring_y = 0.34 if y > 0 else (-0.34 if y < 0 else 0.0)
        _add_member(
            tower_base,
            (x, y, 2.22),
            (ring_x, ring_y, 2.48),
            0.012,
            warning_red,
            name=f"guard_strut_{len(tower_base.visuals):02d}",
        )

    tower_base.inertial = Inertial.from_geometry(
        Box((1.40, 1.40, 2.36)),
        mass=460.0,
        origin=Origin(xyz=(0.0, 0.0, 1.18)),
    )

    pan_stage = model.part("pan_stage")
    pan_stage.visual(
        Cylinder(radius=0.17, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=dark_steel,
        name="turntable_drum",
    )
    pan_stage.visual(
        Cylinder(radius=0.23, length=0.05),
        origin=Origin(xyz=(0.0, 0.0, 0.105)),
        material=steel,
        name="rotating_deck",
    )
    pan_stage.visual(
        Box((0.20, 0.22, 0.12)),
        origin=Origin(xyz=(0.0, 0.0, 0.15)),
        material=painted_gray,
        name="center_pedestal",
    )
    pan_stage.visual(
        Box((0.24, 0.04, 0.46)),
        origin=Origin(xyz=(-0.03, 0.19, 0.31)),
        material=painted_gray,
        name="left_yoke_arm",
    )
    pan_stage.visual(
        Box((0.24, 0.04, 0.46)),
        origin=Origin(xyz=(-0.03, -0.19, 0.31)),
        material=painted_gray,
        name="right_yoke_arm",
    )
    pan_stage.visual(
        Box((0.05, 0.38, 0.06)),
        origin=Origin(xyz=(-0.11, 0.0, 0.55)),
        material=dark_steel,
        name="rear_yoke_tie",
    )
    pan_stage.visual(
        Cylinder(radius=0.032, length=0.020),
        origin=Origin(xyz=(0.0, 0.200, 0.36), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="left_trunnion_bracket",
    )
    pan_stage.visual(
        Cylinder(radius=0.032, length=0.020),
        origin=Origin(xyz=(0.0, -0.200, 0.36), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="right_trunnion_bracket",
    )
    pan_stage.inertial = Inertial.from_geometry(
        Box((0.46, 0.44, 0.60)),
        mass=85.0,
        origin=Origin(xyz=(0.0, 0.0, 0.22)),
    )

    lamp_head = model.part("lamp_head")
    lamp_head.visual(
        Cylinder(radius=0.14, length=0.34),
        origin=Origin(xyz=(0.12, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=housing_black,
        name="lamp_shell",
    )
    lamp_head.visual(
        Cylinder(radius=0.16, length=0.03),
        origin=Origin(xyz=(0.295, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="front_bezel",
    )
    lamp_head.visual(
        Cylinder(radius=0.145, length=0.012),
        origin=Origin(xyz=(0.316, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=lens_glass,
        name="front_lens",
    )
    lamp_head.visual(
        Cylinder(radius=0.10, length=0.12),
        origin=Origin(xyz=(-0.09, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="rear_housing",
    )
    lamp_head.visual(
        Box((0.12, 0.10, 0.06)),
        origin=Origin(xyz=(-0.02, 0.0, 0.16)),
        material=dark_steel,
        name="ballast_box",
    )
    lamp_head.visual(
        Box((0.10, 0.040, 0.080)),
        origin=Origin(xyz=(0.01, 0.120, 0.0)),
        material=dark_steel,
        name="left_trunnion_block",
    )
    lamp_head.visual(
        Box((0.10, 0.040, 0.080)),
        origin=Origin(xyz=(0.01, -0.120, 0.0)),
        material=dark_steel,
        name="right_trunnion_block",
    )
    lamp_head.visual(
        Cylinder(radius=0.022, length=0.040),
        origin=Origin(xyz=(0.0, 0.150, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="left_trunnion_pin",
    )
    lamp_head.visual(
        Cylinder(radius=0.022, length=0.040),
        origin=Origin(xyz=(0.0, -0.150, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="right_trunnion_pin",
    )
    lamp_head.visual(
        Box((0.14, 0.07, 0.04)),
        origin=Origin(xyz=(-0.19, 0.0, -0.08)),
        material=dark_steel,
        name="rear_access_box",
    )
    lamp_head.inertial = Inertial.from_geometry(
        Box((0.48, 0.34, 0.38)),
        mass=42.0,
        origin=Origin(xyz=(0.08, 0.0, 0.02)),
    )

    model.articulation(
        "pan_rotation",
        ArticulationType.REVOLUTE,
        parent=tower_base,
        child=pan_stage,
        origin=Origin(xyz=(0.0, 0.0, 2.31)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=900.0,
            velocity=0.9,
            lower=-math.radians(175.0),
            upper=math.radians(175.0),
        ),
    )
    model.articulation(
        "tilt_rotation",
        ArticulationType.REVOLUTE,
        parent=pan_stage,
        child=lamp_head,
        origin=Origin(xyz=(0.0, 0.0, 0.36)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=450.0,
            velocity=0.8,
            lower=math.radians(-35.0),
            upper=math.radians(70.0),
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

    tower_base = object_model.get_part("tower_base")
    pan_stage = object_model.get_part("pan_stage")
    lamp_head = object_model.get_part("lamp_head")
    pan_joint = object_model.get_articulation("pan_rotation")
    tilt_joint = object_model.get_articulation("tilt_rotation")

    with ctx.pose({pan_joint: 0.0, tilt_joint: 0.0}):
        ctx.expect_gap(
            pan_stage,
            tower_base,
            axis="z",
            positive_elem="turntable_drum",
            negative_elem="bearing_cap",
            max_gap=0.002,
            max_penetration=0.0,
            name="turntable seats on mast bearing",
        )
        ctx.expect_gap(
            pan_stage,
            lamp_head,
            axis="y",
            positive_elem="left_yoke_arm",
            negative_elem="lamp_shell",
            min_gap=0.015,
            max_gap=0.030,
            name="left arm clears lamp shell",
        )
        ctx.expect_gap(
            lamp_head,
            pan_stage,
            axis="y",
            positive_elem="lamp_shell",
            negative_elem="right_yoke_arm",
            min_gap=0.015,
            max_gap=0.030,
            name="right arm clears lamp shell",
        )
        ctx.expect_contact(
            pan_stage,
            lamp_head,
            elem_a="left_yoke_arm",
            elem_b="left_trunnion_pin",
            contact_tol=0.001,
            name="left tilt trunnion pin bears on the yoke arm",
        )
        ctx.expect_contact(
            pan_stage,
            lamp_head,
            elem_a="right_yoke_arm",
            elem_b="right_trunnion_pin",
            contact_tol=0.001,
            name="right tilt trunnion pin bears on the yoke arm",
        )

        rest_lens_center = _aabb_center(ctx.part_element_world_aabb(lamp_head, elem="front_lens"))

    with ctx.pose({pan_joint: 0.0, tilt_joint: math.radians(45.0)}):
        raised_lens_center = _aabb_center(ctx.part_element_world_aabb(lamp_head, elem="front_lens"))

    ctx.check(
        "positive tilt raises the lamp beam",
        rest_lens_center is not None
        and raised_lens_center is not None
        and raised_lens_center[2] > rest_lens_center[2] + 0.12,
        details=f"rest={rest_lens_center}, raised={raised_lens_center}",
    )

    with ctx.pose({pan_joint: math.radians(60.0), tilt_joint: 0.0}):
        panned_lens_center = _aabb_center(ctx.part_element_world_aabb(lamp_head, elem="front_lens"))

    ctx.check(
        "positive pan swings the beam toward positive y",
        rest_lens_center is not None
        and panned_lens_center is not None
        and panned_lens_center[1] > rest_lens_center[1] + 0.20,
        details=f"rest={rest_lens_center}, panned={panned_lens_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
