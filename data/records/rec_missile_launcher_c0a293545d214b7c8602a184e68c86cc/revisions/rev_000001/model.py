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
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _rounded_rect_loop_x(
    x: float,
    *,
    width: float,
    height: float,
    radius: float,
    y_center: float = 0.0,
    z_center: float = 0.0,
    corner_segments: int = 8,
):
    profile = rounded_rect_profile(width, height, radius, corner_segments=corner_segments)
    return tuple((x, y + y_center, z + z_center) for y, z in profile)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="coastal_defense_launcher")

    concrete = model.material("concrete", rgba=(0.62, 0.63, 0.64, 1.0))
    naval_gray = model.material("naval_gray", rgba=(0.63, 0.66, 0.69, 1.0))
    pale_naval_gray = model.material("pale_naval_gray", rgba=(0.73, 0.76, 0.79, 1.0))
    machinery_gray = model.material("machinery_gray", rgba=(0.36, 0.39, 0.42, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.17, 0.19, 0.21, 1.0))
    hatch_black = model.material("hatch_black", rgba=(0.09, 0.10, 0.11, 1.0))

    pod_shell = _mesh(
        "launcher_pod_shell",
        section_loft(
            [
                _rounded_rect_loop_x(
                    -0.55,
                    width=1.42,
                    height=1.18,
                    radius=0.14,
                    z_center=-0.01,
                ),
                _rounded_rect_loop_x(
                    -0.08,
                    width=1.66,
                    height=1.32,
                    radius=0.16,
                    z_center=0.00,
                ),
                _rounded_rect_loop_x(
                    1.55,
                    width=1.76,
                    height=1.42,
                    radius=0.18,
                    z_center=0.02,
                ),
                _rounded_rect_loop_x(
                    3.55,
                    width=1.58,
                    height=1.26,
                    radius=0.16,
                    z_center=0.02,
                ),
                _rounded_rect_loop_x(
                    4.28,
                    width=1.20,
                    height=1.02,
                    radius=0.13,
                    z_center=0.01,
                ),
            ]
        ),
    )

    base = model.part("base")
    base.visual(
        Box((4.80, 4.30, 0.55)),
        origin=Origin(xyz=(0.0, 0.0, 0.275)),
        material=concrete,
        name="foundation_block",
    )
    base.visual(
        Box((2.10, 1.15, 0.95)),
        origin=Origin(xyz=(-1.15, 0.0, 1.025)),
        material=machinery_gray,
        name="service_annex",
    )
    base.visual(
        Box((0.55, 1.60, 0.82)),
        origin=Origin(xyz=(-1.95, 0.0, 0.96)),
        material=machinery_gray,
        name="rear_countermass",
    )
    base.visual(
        Cylinder(radius=0.92, length=1.90),
        origin=Origin(xyz=(0.0, 0.0, 1.50)),
        material=naval_gray,
        name="pedestal_column",
    )
    base.visual(
        Cylinder(radius=1.30, length=0.35),
        origin=Origin(xyz=(0.0, 0.0, 2.625)),
        material=machinery_gray,
        name="slew_housing",
    )
    base.visual(
        Cylinder(radius=1.56, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 2.75)),
        material=dark_metal,
        name="yaw_ring_skirt",
    )
    base.visual(
        Box((1.45, 0.30, 0.48)),
        origin=Origin(xyz=(0.0, 1.05, 0.79)),
        material=machinery_gray,
        name="port_buttress",
    )
    base.visual(
        Box((1.45, 0.30, 0.48)),
        origin=Origin(xyz=(0.0, -1.05, 0.79)),
        material=machinery_gray,
        name="starboard_buttress",
    )
    base.inertial = Inertial.from_geometry(
        Box((4.80, 4.30, 2.80)),
        mass=18000.0,
        origin=Origin(xyz=(0.0, 0.0, 1.40)),
    )

    carriage = model.part("carriage")
    carriage.visual(
        Cylinder(radius=1.38, length=0.30),
        origin=Origin(xyz=(0.0, 0.0, 0.15)),
        material=dark_metal,
        name="turntable_drum",
    )
    carriage.visual(
        Cylinder(radius=1.64, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, 0.36)),
        material=machinery_gray,
        name="rotor_flange",
    )
    carriage.visual(
        Box((3.30, 2.90, 0.38)),
        origin=Origin(xyz=(0.0, 0.0, 0.43)),
        material=naval_gray,
        name="deck_platform",
    )
    carriage.visual(
        Box((0.78, 1.55, 0.34)),
        origin=Origin(xyz=(-1.02, 0.0, 0.78)),
        material=machinery_gray,
        name="rear_drive_housing",
    )
    carriage.visual(
        Box((0.52, 2.20, 0.22)),
        origin=Origin(xyz=(0.62, 0.0, 0.65)),
        material=machinery_gray,
        name="front_crossbeam",
    )
    carriage.visual(
        Box((1.06, 0.18, 1.04)),
        origin=Origin(xyz=(0.06, 1.17, 0.92)),
        material=pale_naval_gray,
        name="left_cheek_lower",
    )
    carriage.visual(
        Box((0.96, 0.18, 0.50)),
        origin=Origin(xyz=(0.14, 1.17, 2.11)),
        material=pale_naval_gray,
        name="left_cheek_upper",
    )
    carriage.visual(
        Box((0.40, 0.18, 1.38)),
        origin=Origin(xyz=(-0.38, 1.17, 1.67)),
        material=machinery_gray,
        name="left_cheek_spine",
    )
    carriage.visual(
        Box((0.24, 0.18, 0.88)),
        origin=Origin(xyz=(0.58, 1.17, 1.64)),
        material=machinery_gray,
        name="left_cheek_front_stanchion",
    )
    carriage.visual(
        Box((1.06, 0.18, 1.04)),
        origin=Origin(xyz=(0.06, -1.17, 0.92)),
        material=pale_naval_gray,
        name="right_cheek_lower",
    )
    carriage.visual(
        Box((0.96, 0.18, 0.50)),
        origin=Origin(xyz=(0.14, -1.17, 2.11)),
        material=pale_naval_gray,
        name="right_cheek_upper",
    )
    carriage.visual(
        Box((0.40, 0.18, 1.38)),
        origin=Origin(xyz=(-0.38, -1.17, 1.67)),
        material=machinery_gray,
        name="right_cheek_spine",
    )
    carriage.visual(
        Box((0.24, 0.18, 0.88)),
        origin=Origin(xyz=(0.58, -1.17, 1.64)),
        material=machinery_gray,
        name="right_cheek_front_stanchion",
    )
    carriage.visual(
        Cylinder(radius=0.30, length=0.18),
        origin=Origin(xyz=(0.10, 1.09, 1.55), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="left_outer_trunnion_bearing",
    )
    carriage.visual(
        Cylinder(radius=0.30, length=0.18),
        origin=Origin(xyz=(0.10, -1.09, 1.55), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="right_outer_trunnion_bearing",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((3.30, 2.90, 2.78)),
        mass=6200.0,
        origin=Origin(xyz=(0.0, 0.0, 1.39)),
    )

    pod = model.part("pod")
    pod.visual(
        pod_shell,
        material=pale_naval_gray,
        name="pod_shell",
    )
    pod.visual(
        Box((0.18, 1.06, 0.90)),
        origin=Origin(xyz=(4.22, 0.0, 0.01)),
        material=machinery_gray,
        name="front_face_panel",
    )
    pod.visual(
        Box((0.42, 1.06, 0.90)),
        origin=Origin(xyz=(-0.62, 0.0, 0.00)),
        material=naval_gray,
        name="rear_service_box",
    )
    pod.visual(
        Cylinder(radius=0.22, length=0.28),
        origin=Origin(xyz=(0.0, 0.86, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="left_trunnion_collar",
    )
    pod.visual(
        Cylinder(radius=0.22, length=0.28),
        origin=Origin(xyz=(0.0, -0.86, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="right_trunnion_collar",
    )
    pod.visual(
        Cylinder(radius=0.15, length=1.44),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="trunnion_tube",
    )
    for name, y_pos, z_pos in (
        ("front_cap_upper_left", 0.36, 0.24),
        ("front_cap_upper_right", -0.36, 0.24),
        ("front_cap_lower_left", 0.36, -0.24),
        ("front_cap_lower_right", -0.36, -0.24),
    ):
        pod.visual(
            Cylinder(radius=0.21, length=0.10),
            origin=Origin(xyz=(4.30, y_pos, z_pos), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=hatch_black,
            name=name,
        )
    pod.visual(
        Box((1.20, 0.36, 0.12)),
        origin=Origin(xyz=(1.40, 0.0, 0.76)),
        material=machinery_gray,
        name="roof_service_hatch",
    )
    pod.inertial = Inertial.from_geometry(
        Box((4.90, 1.90, 1.60)),
        mass=4800.0,
        origin=Origin(xyz=(1.85, 0.0, 0.0)),
    )

    model.articulation(
        "base_yaw",
        ArticulationType.REVOLUTE,
        parent=base,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, 2.80)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=90000.0,
            velocity=0.35,
            lower=-math.pi,
            upper=math.pi,
        ),
    )
    model.articulation(
        "pod_elevation",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=pod,
        origin=Origin(xyz=(0.10, 0.0, 1.55)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=65000.0,
            velocity=0.45,
            lower=0.0,
            upper=1.05,
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

    base = object_model.get_part("base")
    carriage = object_model.get_part("carriage")
    pod = object_model.get_part("pod")
    yaw = object_model.get_articulation("base_yaw")
    elevation = object_model.get_articulation("pod_elevation")

    with ctx.pose({yaw: 0.0, elevation: 0.0}):
        ctx.expect_contact(
            carriage,
            base,
            elem_a="turntable_drum",
            elem_b="slew_housing",
            name="turntable drum seats on the slew housing",
        )
        ctx.expect_contact(
            pod,
            carriage,
            elem_a="left_trunnion_collar",
            elem_b="left_outer_trunnion_bearing",
            name="left trunnion collar bears on the left bearing",
        )
        ctx.expect_contact(
            pod,
            carriage,
            elem_a="right_trunnion_collar",
            elem_b="right_outer_trunnion_bearing",
            name="right trunnion collar bears on the right bearing",
        )
        ctx.expect_gap(
            pod,
            carriage,
            axis="z",
            positive_elem="pod_shell",
            negative_elem="deck_platform",
            min_gap=0.20,
            name="pod shell clears the deck in the travel position",
        )

        rest_front = ctx.part_element_world_aabb(pod, elem="front_face_panel")
        elevated_front = None
        yawed_front = None

        with ctx.pose({elevation: elevation.motion_limits.upper}):
            elevated_front = ctx.part_element_world_aabb(pod, elem="front_face_panel")

        with ctx.pose({yaw: math.pi / 2.0}):
            yawed_front = ctx.part_element_world_aabb(pod, elem="front_face_panel")

    rest_center = None
    elevated_center = None
    yawed_center = None
    if rest_front is not None:
        rest_center = tuple((low + high) * 0.5 for low, high in zip(rest_front[0], rest_front[1]))
    if elevated_front is not None:
        elevated_center = tuple((low + high) * 0.5 for low, high in zip(elevated_front[0], elevated_front[1]))
    if yawed_front is not None:
        yawed_center = tuple((low + high) * 0.5 for low, high in zip(yawed_front[0], yawed_front[1]))

    ctx.check(
        "positive elevation lifts the pod nose",
        rest_center is not None
        and elevated_center is not None
        and elevated_center[2] > rest_center[2] + 1.0,
        details=f"rest_center={rest_center}, elevated_center={elevated_center}",
    )
    ctx.check(
        "positive yaw swings the pod from +x toward +y",
        rest_center is not None
        and yawed_center is not None
        and rest_center[0] > 3.0
        and abs(yawed_center[0]) < 0.7
        and yawed_center[1] > 3.0,
        details=f"rest_center={rest_center}, yawed_center={yawed_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
