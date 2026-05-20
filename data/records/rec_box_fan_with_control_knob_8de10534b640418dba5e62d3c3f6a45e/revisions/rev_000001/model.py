from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    ExtrudeGeometry,
    LatheGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    wire_from_points,
)


def _circle_path_y(radius: float, y: float, segments: int = 40) -> list[tuple[float, float, float]]:
    return [
        (
            radius * math.cos((2.0 * math.pi * i) / segments),
            y,
            radius * math.sin((2.0 * math.pi * i) / segments),
        )
        for i in range(segments)
    ]


def _spiral_path_y(
    *,
    y: float,
    start_radius: float,
    end_radius: float,
    turns: float,
    samples: int,
) -> list[tuple[float, float, float]]:
    points: list[tuple[float, float, float]] = []
    for index in range(samples + 1):
        t = index / samples
        angle = turns * 2.0 * math.pi * t
        radius = start_radius + (end_radius - start_radius) * t
        points.append((radius * math.cos(angle), y, radius * math.sin(angle)))
    return points


def _build_spiral_grille(
    *,
    y: float,
    start_radius: float,
    end_radius: float,
    turns: float,
    wire_radius: float,
    samples: int = 220,
) -> BoxGeometry:
    return wire_from_points(
        _spiral_path_y(
            y=y,
            start_radius=start_radius,
            end_radius=end_radius,
            turns=turns,
            samples=samples,
        ),
        radius=wire_radius,
        cap_ends=True,
        corner_mode="miter",
    )


def _build_housing_shell_mesh() -> BoxGeometry:
    outer_profile = [
        (0.088, -0.041),
        (0.104, -0.036),
        (0.111, -0.010),
        (0.113, 0.020),
        (0.109, 0.041),
    ]
    inner_profile = [
        (0.081, -0.035),
        (0.091, -0.030),
        (0.096, -0.008),
        (0.097, 0.018),
        (0.094, 0.035),
    ]

    shell = LatheGeometry.from_shell_profiles(outer_profile, inner_profile, segments=72)
    shell.rotate_x(math.pi / 2.0)
    return shell


def _build_blade_mesh() -> BoxGeometry:
    rotor = CylinderGeometry(radius=0.021, height=0.026)
    rotor.rotate_x(math.pi / 2.0)

    for angle in (0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0):
        blade = BoxGeometry((0.078, 0.007, 0.030))
        blade.rotate_x(0.35)
        blade.translate(0.040, 0.0, 0.0)
        blade.rotate_y(angle)
        rotor.merge(blade)

    return rotor


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="desk_personal_box_fan")

    charcoal = model.material("charcoal", rgba=(0.16, 0.17, 0.19, 1.0))
    dark_gray = model.material("dark_gray", rgba=(0.25, 0.27, 0.29, 1.0))
    warm_gray = model.material("warm_gray", rgba=(0.73, 0.75, 0.77, 1.0))
    off_white = model.material("off_white", rgba=(0.90, 0.91, 0.92, 1.0))
    red = model.material("red", rgba=(0.70, 0.15, 0.12, 1.0))

    base = model.part("base")
    base_shell = ExtrudeGeometry.from_z0(rounded_rect_profile(0.182, 0.132, 0.024), 0.018)
    base.visual(mesh_from_geometry(base_shell, "base_shell"), material=charcoal, name="base_shell")
    base.visual(
        Cylinder(radius=0.020, length=0.034),
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        material=dark_gray,
        name="base_pedestal",
    )

    swivel_yoke = model.part("swivel_yoke")
    swivel_yoke.visual(
        Cylinder(radius=0.028, length=0.012),
        origin=Origin(xyz=(0.0, -0.028, 0.006)),
        material=dark_gray,
        name="yoke_turret",
    )
    swivel_yoke.visual(
        Box((0.036, 0.020, 0.132)),
        origin=Origin(xyz=(0.0, -0.056, 0.076)),
        material=dark_gray,
        name="yoke_spine",
    )
    swivel_yoke.visual(
        Box((0.278, 0.016, 0.020)),
        origin=Origin(xyz=(0.0, -0.056, 0.150)),
        material=dark_gray,
        name="yoke_crossbar",
    )
    swivel_yoke.visual(
        Box((0.020, 0.076, 0.120)),
        origin=Origin(xyz=(-0.131, -0.028, 0.118)),
        material=dark_gray,
        name="left_arm",
    )
    swivel_yoke.visual(
        Box((0.020, 0.076, 0.120)),
        origin=Origin(xyz=(0.131, -0.028, 0.118)),
        material=dark_gray,
        name="right_arm",
    )

    housing = model.part("housing")
    housing.visual(
        mesh_from_geometry(_build_housing_shell_mesh(), "housing_shell"),
        material=warm_gray,
        name="housing_shell",
    )
    housing.visual(
        mesh_from_geometry(
            _build_spiral_grille(
                y=0.036,
                start_radius=0.026,
                end_radius=0.096,
                turns=3.25,
                wire_radius=0.0023,
            ),
            "front_grille",
        ),
        material=warm_gray,
        name="front_grille",
    )
    housing.visual(
        mesh_from_geometry(
            _build_spiral_grille(
                y=-0.036,
                start_radius=0.030,
                end_radius=0.091,
                turns=2.75,
                wire_radius=0.0025,
            ),
            "rear_grille",
        ),
        material=warm_gray,
        name="rear_grille",
    )
    housing.visual(
        Cylinder(radius=0.029, length=0.030),
        origin=Origin(xyz=(0.0, -0.024, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_gray,
        name="motor_pod",
    )
    housing.visual(
        Box((0.014, 0.018, 0.078)),
        origin=Origin(xyz=(0.040, -0.022, 0.040), rpy=(0.0, math.pi / 4.0, 0.0)),
        material=dark_gray,
        name="strut_ne",
    )
    housing.visual(
        Box((0.014, 0.018, 0.078)),
        origin=Origin(xyz=(-0.040, -0.022, 0.040), rpy=(0.0, -math.pi / 4.0, 0.0)),
        material=dark_gray,
        name="strut_nw",
    )
    housing.visual(
        Box((0.014, 0.018, 0.078)),
        origin=Origin(xyz=(-0.040, -0.022, -0.040), rpy=(0.0, math.pi / 4.0, 0.0)),
        material=dark_gray,
        name="strut_sw",
    )
    housing.visual(
        Box((0.014, 0.018, 0.078)),
        origin=Origin(xyz=(0.040, -0.022, -0.040), rpy=(0.0, -math.pi / 4.0, 0.0)),
        material=dark_gray,
        name="strut_se",
    )
    housing.visual(
        Cylinder(radius=0.018, length=0.010),
        origin=Origin(xyz=(-0.116, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_gray,
        name="pivot_left",
    )
    housing.visual(
        Cylinder(radius=0.018, length=0.010),
        origin=Origin(xyz=(0.116, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_gray,
        name="pivot_right",
    )

    blade = model.part("blade")
    blade.visual(
        mesh_from_geometry(_build_blade_mesh(), "blade_mesh"),
        material=off_white,
        name="blade_mesh",
    )
    blade.visual(
        Cylinder(radius=0.0045, length=0.014),
        origin=Origin(xyz=(0.0, -0.012, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_gray,
        name="blade_axle",
    )
    blade.visual(
        Sphere(radius=0.009),
        origin=Origin(xyz=(0.0, 0.012, 0.0)),
        material=off_white,
        name="blade_spinner",
    )
    blade.visual(
        Box((0.012, 0.004, 0.008)),
        origin=Origin(xyz=(0.068, 0.001, 0.0)),
        material=off_white,
        name="blade_marker",
    )

    knob = model.part("speed_knob")
    knob.visual(
        Cylinder(radius=0.014, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=dark_gray,
        name="knob_body",
    )
    knob.visual(
        Cylinder(radius=0.009, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=charcoal,
        name="knob_cap",
    )
    knob.visual(
        Box((0.006, 0.010, 0.003)),
        origin=Origin(xyz=(0.010, 0.0, 0.0155)),
        material=red,
        name="knob_pointer",
    )

    model.articulation(
        "base_to_swivel",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=swivel_yoke,
        origin=Origin(xyz=(0.0, 0.0, 0.052)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=4.0),
    )
    model.articulation(
        "swivel_to_housing",
        ArticulationType.REVOLUTE,
        parent=swivel_yoke,
        child=housing,
        origin=Origin(xyz=(0.0, 0.0, 0.128)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=2.0, lower=-0.55, upper=0.75),
    )
    model.articulation(
        "housing_to_blade",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=blade,
        origin=Origin(xyz=(0.0, 0.010, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.5, velocity=40.0),
    )
    model.articulation(
        "base_to_knob",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=knob,
        origin=Origin(xyz=(0.055, 0.030, 0.018)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.2, velocity=8.0),
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
    swivel_yoke = object_model.get_part("swivel_yoke")
    housing = object_model.get_part("housing")
    blade = object_model.get_part("blade")
    knob = object_model.get_part("speed_knob")

    oscillation = object_model.get_articulation("base_to_swivel")
    tilt = object_model.get_articulation("swivel_to_housing")
    blade_spin = object_model.get_articulation("housing_to_blade")
    knob_spin = object_model.get_articulation("base_to_knob")

    def center_of(aabb):
        if aabb is None:
            return None
        lo, hi = aabb
        return tuple((lo[i] + hi[i]) * 0.5 for i in range(3))

    ctx.check(
        "oscillation axis is vertical",
        tuple(oscillation.axis) == (0.0, 0.0, 1.0),
        details=f"axis={oscillation.axis}",
    )
    ctx.check(
        "tilt axis is horizontal",
        tuple(tilt.axis) == (1.0, 0.0, 0.0),
        details=f"axis={tilt.axis}",
    )
    ctx.check(
        "blade spins about airflow axis",
        tuple(blade_spin.axis) == (0.0, 1.0, 0.0),
        details=f"axis={blade_spin.axis}",
    )
    ctx.check(
        "speed knob spins vertically",
        tuple(knob_spin.axis) == (0.0, 0.0, 1.0),
        details=f"axis={knob_spin.axis}",
    )

    with ctx.pose({oscillation: 0.0, tilt: 0.0, blade_spin: 0.0, knob_spin: 0.0}):
        ctx.expect_contact(
            swivel_yoke,
            base,
            elem_a="yoke_turret",
            elem_b="base_pedestal",
            contact_tol=0.0015,
            name="swivel sits on base pedestal",
        )
        ctx.expect_contact(
            housing,
            swivel_yoke,
            elem_a="pivot_left",
            elem_b="left_arm",
            contact_tol=0.0015,
            name="left trunnion meets yoke arm",
        )
        ctx.expect_contact(
            blade,
            housing,
            elem_a="blade_axle",
            elem_b="motor_pod",
            contact_tol=0.0015,
            name="blade axle seats against motor pod",
        )
        ctx.expect_contact(
            knob,
            base,
            elem_a="knob_body",
            elem_b="base_shell",
            contact_tol=0.0015,
            name="speed knob mounts on base",
        )
        ctx.expect_gap(
            housing,
            blade,
            axis="y",
            positive_elem="front_grille",
            negative_elem="blade_mesh",
            min_gap=0.008,
            name="front grille clears blade",
        )
        ctx.expect_gap(
            blade,
            housing,
            axis="y",
            positive_elem="blade_mesh",
            negative_elem="rear_grille",
            min_gap=0.008,
            name="rear cage clears blade",
        )

        rest_spinner = center_of(ctx.part_element_world_aabb(blade, elem="blade_spinner"))
        rest_blade_marker = center_of(ctx.part_element_world_aabb(blade, elem="blade_marker"))
        rest_knob_pointer = center_of(ctx.part_element_world_aabb(knob, elem="knob_pointer"))

    with ctx.pose(base_to_swivel=math.pi / 2.0):
        swung_spinner = center_of(ctx.part_element_world_aabb(blade, elem="blade_spinner"))
    ctx.check(
        "oscillation turns housing in plan",
        rest_spinner is not None
        and swung_spinner is not None
        and abs(swung_spinner[0] - rest_spinner[0]) > 0.020
        and abs(swung_spinner[1]) < abs(rest_spinner[1]),
        details=f"rest={rest_spinner}, swung={swung_spinner}",
    )

    with ctx.pose(swivel_to_housing=0.35):
        tilted_spinner = center_of(ctx.part_element_world_aabb(blade, elem="blade_spinner"))
    ctx.check(
        "positive tilt raises front of housing",
        rest_spinner is not None
        and tilted_spinner is not None
        and tilted_spinner[2] > rest_spinner[2] + 0.007,
        details=f"rest={rest_spinner}, tilted={tilted_spinner}",
    )

    with ctx.pose(housing_to_blade=math.pi / 2.0):
        spun_blade_marker = center_of(ctx.part_element_world_aabb(blade, elem="blade_marker"))
    ctx.check(
        "blade marker moves around spin axis",
        rest_blade_marker is not None
        and spun_blade_marker is not None
        and abs(spun_blade_marker[0] - rest_blade_marker[0]) > 0.030
        and abs(spun_blade_marker[2] - rest_blade_marker[2]) > 0.030,
        details=f"rest={rest_blade_marker}, spun={spun_blade_marker}",
    )

    with ctx.pose(base_to_knob=math.pi / 2.0):
        turned_knob_pointer = center_of(ctx.part_element_world_aabb(knob, elem="knob_pointer"))
    ctx.check(
        "knob pointer rotates around knob axis",
        rest_knob_pointer is not None
        and turned_knob_pointer is not None
        and abs(turned_knob_pointer[0] - rest_knob_pointer[0]) > 0.006
        and abs(turned_knob_pointer[1] - rest_knob_pointer[1]) > 0.006,
        details=f"rest={rest_knob_pointer}, turned={turned_knob_pointer}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
