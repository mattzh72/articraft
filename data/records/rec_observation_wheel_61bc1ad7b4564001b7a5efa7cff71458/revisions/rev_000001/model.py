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
    superellipse_side_loft,
    tube_from_spline_points,
)


POD_COUNT = 8
HUB_HEIGHT = 10.3
WHEEL_RADIUS = 6.1
RIM_HALF_SPAN = 0.55
PIVOT_RADIUS = 7.85


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _circle_point(radius: float, angle: float, *, y: float = 0.0) -> tuple[float, float, float]:
    return (radius * math.cos(angle), y, radius * math.sin(angle))


def _tube(
    name: str,
    points: list[tuple[float, float, float]],
    *,
    radius: float,
    samples_per_segment: int = 2,
    radial_segments: int = 18,
):
    return mesh_from_geometry(
        tube_from_spline_points(
            points,
            radius=radius,
            samples_per_segment=samples_per_segment,
            radial_segments=radial_segments,
            cap_ends=True,
        ),
        name,
    )


def _add_leg(
    part,
    *,
    name: str,
    lower: tuple[float, float, float],
    upper: tuple[float, float, float],
    size_x: float,
    size_y: float,
    material,
):
    dx = upper[0] - lower[0]
    dz = upper[2] - lower[2]
    length = math.hypot(dx, dz)
    angle = math.atan2(dx, dz)
    part.visual(
        Box((size_x, size_y, length)),
        origin=Origin(
            xyz=(
                (lower[0] + upper[0]) * 0.5,
                (lower[1] + upper[1]) * 0.5,
                (lower[2] + upper[2]) * 0.5,
            ),
            rpy=(0.0, angle, 0.0),
        ),
        material=material,
        name=name,
    )


def _build_pod_part(model: ArticulatedObject, name: str, *, shell_mesh, shell_material, glass_material, metal_material):
    pod = model.part(name)
    pod.visual(
        Cylinder(radius=0.05, length=0.772),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal_material,
        name="pivot_beam",
    )
    pod.visual(
        Box((0.08, 0.08, 0.32)),
        origin=Origin(xyz=(0.0, 0.22, -0.16)),
        material=metal_material,
        name="hanger_front",
    )
    pod.visual(
        Box((0.08, 0.08, 0.32)),
        origin=Origin(xyz=(0.0, -0.22, -0.16)),
        material=metal_material,
        name="hanger_rear",
    )
    pod.visual(
        Box((0.22, 0.56, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, -0.30)),
        material=metal_material,
        name="hanger_crossmember",
    )
    pod.visual(shell_mesh, material=shell_material, name="pod_shell")
    pod.visual(
        Box((0.94, 0.64, 0.72)),
        origin=Origin(xyz=(0.0, 0.0, -0.92)),
        material=glass_material,
        name="window_band",
    )
    pod.visual(
        Box((0.72, 0.60, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, -1.535)),
        material=metal_material,
        name="pod_floor",
    )
    pod.inertial = Inertial.from_geometry(
        Box((1.20, 0.90, 1.60)),
        mass=240.0,
        origin=Origin(xyz=(0.0, 0.0, -0.92)),
    )
    return pod


def _axis_close(axis: tuple[float, float, float] | None, expected: tuple[float, float, float], tol: float = 1e-6) -> bool:
    if axis is None:
        return False
    return all(abs(a - b) <= tol for a, b in zip(axis, expected))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="observation_wheel")

    frame_white = model.material("frame_white", rgba=(0.92, 0.93, 0.95, 1.0))
    machinery_gray = model.material("machinery_gray", rgba=(0.42, 0.44, 0.47, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.15, 0.16, 0.18, 1.0))
    deck_gray = model.material("deck_gray", rgba=(0.50, 0.51, 0.54, 1.0))
    gondola_white = model.material("gondola_white", rgba=(0.96, 0.97, 0.98, 1.0))
    glass_blue = model.material("glass_blue", rgba=(0.24, 0.39, 0.55, 0.72))

    rim_mesh = _mesh(
        "wheel_rim",
        TorusGeometry(radius=WHEEL_RADIUS, tube=0.14, radial_segments=24, tubular_segments=112).rotate_x(math.pi / 2.0),
    )
    pod_shell_mesh = _mesh(
        "gondola_shell",
        superellipse_side_loft(
            [
                (-0.34, -1.56, -0.40, 0.86),
                (-0.18, -1.60, -0.31, 1.02),
                (0.00, -1.62, -0.27, 1.12),
                (0.18, -1.60, -0.31, 1.02),
                (0.34, -1.56, -0.40, 0.86),
            ],
            exponents=2.2,
            segments=52,
        ),
    )
    support = model.part("support_frame")
    support.visual(
        Box((10.8, 3.4, 0.55)),
        origin=Origin(xyz=(0.0, 0.0, 0.275)),
        material=deck_gray,
        name="base_deck",
    )
    support.visual(
        Box((7.8, 0.44, 0.16)),
        origin=Origin(xyz=(0.0, 0.0, 0.63)),
        material=dark_steel,
        name="base_spine",
    )
    support.visual(
        Box((2.20, 0.95, 1.15)),
        origin=Origin(xyz=(0.0, 1.15, 1.125)),
        material=machinery_gray,
        name="drive_house",
    )
    _add_leg(
        support,
        name="left_leg",
        lower=(-3.15, 0.0, 0.55),
        upper=(-0.65, 0.0, 8.20),
        size_x=0.46,
        size_y=0.78,
        material=frame_white,
    )
    _add_leg(
        support,
        name="right_leg",
        lower=(3.15, 0.0, 0.55),
        upper=(0.65, 0.0, 8.20),
        size_x=0.46,
        size_y=0.78,
        material=frame_white,
    )
    support.visual(
        Box((0.50, 1.66, 0.40)),
        origin=Origin(xyz=(-0.47, 0.0, 8.00)),
        material=frame_white,
        name="crown_tie_left",
    )
    support.visual(
        Box((0.50, 1.66, 0.40)),
        origin=Origin(xyz=(0.47, 0.0, 8.00)),
        material=frame_white,
        name="crown_tie_right",
    )
    support.visual(
        Box((0.55, 0.24, 1.90)),
        origin=Origin(xyz=(0.0, 0.795, 9.15)),
        material=frame_white,
        name="front_bearing_post",
    )
    support.visual(
        Box((0.55, 0.24, 1.90)),
        origin=Origin(xyz=(0.0, -0.795, 9.15)),
        material=frame_white,
        name="rear_bearing_post",
    )
    support.visual(
        Box((0.85, 0.24, 0.40)),
        origin=Origin(xyz=(0.0, 0.795, 10.24)),
        material=machinery_gray,
        name="front_bearing_block",
    )
    support.visual(
        Box((0.85, 0.24, 0.40)),
        origin=Origin(xyz=(0.0, -0.795, 10.24)),
        material=machinery_gray,
        name="rear_bearing_block",
    )
    support.inertial = Inertial.from_geometry(
        Box((10.8, 3.4, 9.0)),
        mass=36000.0,
        origin=Origin(xyz=(0.0, 0.0, 4.5)),
    )

    wheel = model.part("wheel")
    wheel.visual(
        Cylinder(radius=0.22, length=1.35),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="axle_shaft",
    )
    wheel.visual(
        Cylinder(radius=0.68, length=1.10),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=machinery_gray,
        name="hub_drum",
    )
    wheel.visual(
        Cylinder(radius=1.04, length=0.14),
        origin=Origin(xyz=(0.0, 0.48, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=machinery_gray,
        name="front_hub_flange",
    )
    wheel.visual(
        Cylinder(radius=1.04, length=0.14),
        origin=Origin(xyz=(0.0, -0.48, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=machinery_gray,
        name="rear_hub_flange",
    )
    wheel.visual(
        rim_mesh,
        origin=Origin(xyz=(0.0, RIM_HALF_SPAN, 0.0)),
        material=frame_white,
        name="front_rim",
    )
    wheel.visual(
        rim_mesh,
        origin=Origin(xyz=(0.0, -RIM_HALF_SPAN, 0.0)),
        material=frame_white,
        name="rear_rim",
    )

    for index in range(POD_COUNT):
        angle = (-math.pi / 2.0) + (2.0 * math.pi * index) / POD_COUNT

        wheel.visual(
            Cylinder(radius=0.08, length=1.10),
            origin=Origin(
                xyz=_circle_point(WHEEL_RADIUS, angle),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=machinery_gray,
            name=f"rim_tie_{index:02d}",
        )

        front_spoke = _tube(
            f"front_spoke_{index:02d}",
            [
                _circle_point(0.98, angle, y=0.48),
                _circle_point(6.10, angle, y=0.55),
            ],
            radius=0.055,
        )
        rear_spoke = _tube(
            f"rear_spoke_{index:02d}",
            [
                _circle_point(0.98, angle, y=-0.48),
                _circle_point(6.10, angle, y=-0.55),
            ],
            radius=0.055,
        )
        front_bracket_arm = _tube(
            f"front_bracket_arm_{index:02d}",
            [
                _circle_point(WHEEL_RADIUS, angle, y=0.44),
                _circle_point(PIVOT_RADIUS, angle, y=0.44),
            ],
            radius=0.055,
        )
        rear_bracket_arm = _tube(
            f"rear_bracket_arm_{index:02d}",
            [
                _circle_point(WHEEL_RADIUS, angle, y=-0.44),
                _circle_point(PIVOT_RADIUS, angle, y=-0.44),
            ],
            radius=0.055,
        )
        if index == 0:
            wheel.visual(front_spoke, material=frame_white, name="front_spoke_00")
            wheel.visual(rear_spoke, material=frame_white, name="rear_spoke_00")
            wheel.visual(front_bracket_arm, material=machinery_gray, name="front_bracket_arm_00")
            wheel.visual(rear_bracket_arm, material=machinery_gray, name="rear_bracket_arm_00")
        else:
            wheel.visual(front_spoke, material=frame_white, name=f"front_spoke_{index:02d}")
            wheel.visual(rear_spoke, material=frame_white, name=f"rear_spoke_{index:02d}")
            wheel.visual(front_bracket_arm, material=machinery_gray, name=f"front_bracket_arm_{index:02d}")
            wheel.visual(rear_bracket_arm, material=machinery_gray, name=f"rear_bracket_arm_{index:02d}")

    wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=6.3, length=1.35),
        mass=8800.0,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
    )

    model.articulation(
        "support_to_wheel",
        ArticulationType.CONTINUOUS,
        parent=support,
        child=wheel,
        origin=Origin(xyz=(0.0, 0.0, HUB_HEIGHT)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=150000.0, velocity=0.80),
    )

    for index in range(POD_COUNT):
        part_name = f"pod_{index:02d}"
        joint_name = f"wheel_to_pod_{index:02d}"
        angle = (-math.pi / 2.0) + (2.0 * math.pi * index) / POD_COUNT
        pod = _build_pod_part(
            model,
            part_name,
            shell_mesh=pod_shell_mesh,
            shell_material=gondola_white,
            glass_material=glass_blue,
            metal_material=dark_steel,
        )
        model.articulation(
            joint_name,
            ArticulationType.CONTINUOUS,
            parent=wheel,
            child=pod,
            origin=Origin(xyz=_circle_point(PIVOT_RADIUS, angle)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=5000.0, velocity=2.0),
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

    support = object_model.get_part("support_frame")
    wheel = object_model.get_part("wheel")
    wheel_spin = object_model.get_articulation("support_to_wheel")
    bottom_pod = object_model.get_part("pod_00")
    bottom_hanger = object_model.get_articulation("wheel_to_pod_00")

    ctx.check(
        "wheel articulation is continuous on the horizontal hub axis",
        wheel_spin.articulation_type == ArticulationType.CONTINUOUS and _axis_close(wheel_spin.axis, (0.0, 1.0, 0.0)),
        details=f"type={wheel_spin.articulation_type}, axis={wheel_spin.axis}",
    )
    ctx.expect_contact(
        support,
        wheel,
        elem_a="front_bearing_post",
        elem_b="axle_shaft",
        name="front bearing post meets the axle shaft",
    )
    ctx.expect_contact(
        support,
        wheel,
        elem_a="rear_bearing_post",
        elem_b="axle_shaft",
        name="rear bearing post meets the axle shaft",
    )

    for index in range(POD_COUNT):
        pod = object_model.get_part(f"pod_{index:02d}")
        hanger = object_model.get_articulation(f"wheel_to_pod_{index:02d}")
        ctx.check(
            f"pod_{index:02d} exists with a horizontal hanger pivot",
            pod is not None
            and hanger.articulation_type == ArticulationType.CONTINUOUS
            and _axis_close(hanger.axis, (0.0, 1.0, 0.0)),
            details=f"type={hanger.articulation_type}, axis={hanger.axis}",
        )

    ctx.expect_gap(
        bottom_pod,
        support,
        axis="z",
        min_gap=0.08,
        positive_elem="pod_shell",
        negative_elem="base_deck",
        name="lowest gondola clears the broad base deck",
    )

    bottom_pivot = ctx.part_world_position(bottom_pod)
    bottom_shell = ctx.part_element_world_aabb(bottom_pod, elem="pod_shell")
    ctx.check(
        "gondola shell hangs beneath its hanger pivot",
        bottom_pivot is not None
        and bottom_shell is not None
        and bottom_shell[1][2] < bottom_pivot[2] - 0.20,
        details=f"pivot={bottom_pivot}, shell_aabb={bottom_shell}",
    )
    ctx.expect_contact(
        bottom_pod,
        wheel,
        elem_a="pivot_beam",
        elem_b="front_bracket_arm_00",
        name="lowest gondola pivot beam meets the front rim bracket",
    )
    ctx.expect_contact(
        bottom_pod,
        wheel,
        elem_a="pivot_beam",
        elem_b="rear_bracket_arm_00",
        name="lowest gondola pivot beam meets the rear rim bracket",
    )

    bottom_pod_rest = ctx.part_world_position(bottom_pod)
    with ctx.pose({wheel_spin: math.pi / 2.0}):
        bottom_pod_quarter_turn = ctx.part_world_position(bottom_pod)
    ctx.check(
        "wheel rotation carries a gondola around the rim",
        bottom_pod_rest is not None
        and bottom_pod_quarter_turn is not None
        and bottom_pod_quarter_turn[0] < bottom_pod_rest[0] - 5.0
        and bottom_pod_quarter_turn[2] > bottom_pod_rest[2] + 5.0,
        details=f"rest={bottom_pod_rest}, quarter_turn={bottom_pod_quarter_turn}",
    )

    shell_rest = ctx.part_element_world_aabb(bottom_pod, elem="pod_shell")
    with ctx.pose({bottom_hanger: math.pi / 2.0}):
        shell_swung = ctx.part_element_world_aabb(bottom_pod, elem="pod_shell")
    rest_center_x = None if shell_rest is None else (shell_rest[0][0] + shell_rest[1][0]) * 0.5
    swung_center_x = None if shell_swung is None else (shell_swung[0][0] + shell_swung[1][0]) * 0.5
    ctx.check(
        "gondola hanger pivot swings the cabin around its axle",
        rest_center_x is not None and swung_center_x is not None and swung_center_x < rest_center_x - 0.60,
        details=f"rest_center_x={rest_center_x}, swung_center_x={swung_center_x}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
