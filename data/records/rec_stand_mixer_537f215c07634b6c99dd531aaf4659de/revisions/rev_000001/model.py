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
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
    tube_from_spline_points,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _xy_section(width: float, depth: float, radius: float, z: float):
    return [(x, y, z) for x, y in rounded_rect_profile(width, depth, radius)]


def _yz_section(width: float, height: float, radius: float, x: float):
    return [(x, y, z) for z, y in rounded_rect_profile(height, width, radius)]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="upscale_stand_mixer")

    body_paint = model.material("body_paint", rgba=(0.94, 0.90, 0.84, 1.0))
    trim_dark = model.material("trim_dark", rgba=(0.14, 0.14, 0.15, 1.0))
    stainless = model.material("stainless", rgba=(0.88, 0.89, 0.90, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.32, 0.34, 0.36, 1.0))
    rubber = model.material("rubber", rgba=(0.07, 0.07, 0.08, 1.0))

    base = model.part("base")

    base.visual(
        _save_mesh(
            "mixer_base_plinth",
            ExtrudeGeometry(rounded_rect_profile(0.36, 0.24, 0.052), 0.04),
        ),
        origin=Origin(xyz=(0.10, 0.0, 0.02)),
        material=body_paint,
        name="base_plinth",
    )

    pedestal_shell = section_loft(
        [
            _xy_section(0.112, 0.132, 0.034, 0.04),
            _xy_section(0.094, 0.116, 0.032, 0.16),
            _xy_section(0.076, 0.102, 0.028, 0.28),
            _xy_section(0.064, 0.096, 0.024, 0.304),
        ]
    )
    base.visual(
        _save_mesh("mixer_pedestal_shell", pedestal_shell),
        origin=Origin(xyz=(-0.046, 0.0, 0.0)),
        material=body_paint,
        name="pedestal_shell",
    )
    base.visual(
        Box((0.066, 0.108, 0.020)),
        origin=Origin(xyz=(-0.021, 0.0, 0.314)),
        material=body_paint,
        name="hinge_saddle",
    )

    for side, visual_name in ((-1.0, "left_slide_rail"), (1.0, "right_slide_rail")):
        base.visual(
            Box((0.124, 0.022, 0.014)),
            origin=Origin(xyz=(0.096, side * 0.037, 0.047)),
            material=trim_dark,
            name=visual_name,
        )

    base.visual(
        Box((0.050, 0.020, 0.008)),
        origin=Origin(xyz=(-0.006, 0.079, 0.044)),
        material=trim_dark,
        name="lock_slider_guide",
    )
    base.visual(
        Box((0.050, 0.020, 0.028)),
        origin=Origin(xyz=(0.004, 0.076, 0.118)),
        material=body_paint,
        name="speed_control_pod",
    )
    base.visual(
        Box((0.034, 0.028, 0.030)),
        origin=Origin(xyz=(-0.008, 0.056, 0.118)),
        material=body_paint,
        name="speed_control_bridge",
    )

    for sx in (-0.03, 0.21):
        for sy in (-0.085, 0.085):
            base.visual(
                Cylinder(radius=0.013, length=0.010),
                origin=Origin(xyz=(sx, sy, 0.005)),
                material=rubber,
            )

    base.inertial = Inertial.from_geometry(
        Box((0.38, 0.25, 0.34)),
        mass=11.0,
        origin=Origin(xyz=(0.08, 0.0, 0.17)),
    )

    carriage = model.part("carriage")
    carriage.visual(
        _save_mesh(
            "mixer_carriage_plate",
            ExtrudeGeometry(rounded_rect_profile(0.165, 0.132, 0.020), 0.010),
        ),
        origin=Origin(xyz=(0.080, 0.0, 0.013)),
        material=trim_dark,
        name="carriage_plate",
    )
    for side, visual_name in ((-1.0, "left_runner"), (1.0, "right_runner")):
        carriage.visual(
            Box((0.094, 0.016, 0.008)),
            origin=Origin(xyz=(0.056, side * 0.037, 0.004)),
            material=dark_metal,
            name=visual_name,
        )
        carriage.visual(
            Box((0.060, 0.018, 0.020)),
            origin=Origin(xyz=(0.092, side * 0.036, 0.022)),
            material=trim_dark,
            name=f"{visual_name}_gusset",
        )
    carriage.visual(
        Cylinder(radius=0.032, length=0.026),
        origin=Origin(xyz=(0.110, 0.0, 0.031)),
        material=trim_dark,
        name="bowl_pedestal",
    )
    carriage.visual(
        Box((0.024, 0.060, 0.014)),
        origin=Origin(xyz=(0.150, 0.0, 0.015)),
        material=trim_dark,
        name="carriage_front_lip",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((0.17, 0.14, 0.06)),
        mass=1.8,
        origin=Origin(xyz=(0.082, 0.0, 0.030)),
    )

    bowl = model.part("bowl")
    bowl.visual(
        _save_mesh(
            "mixer_bowl_shell",
            LatheGeometry.from_shell_profiles(
                [
                    (0.030, 0.000),
                    (0.070, 0.014),
                    (0.108, 0.055),
                    (0.129, 0.108),
                    (0.132, 0.152),
                    (0.138, 0.172),
                ],
                [
                    (0.000, 0.008),
                    (0.056, 0.018),
                    (0.101, 0.057),
                    (0.122, 0.109),
                    (0.124, 0.165),
                ],
                segments=60,
                end_cap="round",
                lip_samples=8,
            ),
        ),
        material=stainless,
        name="bowl_shell",
    )
    bowl.visual(
        Cylinder(radius=0.034, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=stainless,
        name="bowl_foot",
    )
    bowl.inertial = Inertial.from_geometry(
        Cylinder(radius=0.14, length=0.18),
        mass=1.2,
        origin=Origin(xyz=(0.0, 0.0, 0.09)),
    )

    head = model.part("head")
    head.visual(
        _save_mesh(
            "mixer_head_shell",
            section_loft(
                [
                    _yz_section(0.100, 0.096, 0.030, 0.018),
                    _yz_section(0.150, 0.146, 0.050, 0.108),
                    _yz_section(0.146, 0.128, 0.046, 0.230),
                    _yz_section(0.106, 0.088, 0.032, 0.342),
                ]
            ),
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=body_paint,
        name="head_shell",
    )
    head.visual(
        Box((0.078, 0.106, 0.022)),
        origin=Origin(xyz=(0.162, 0.0, -0.042)),
        material=body_paint,
        name="nose_housing",
    )
    head.visual(
        Cylinder(radius=0.021, length=0.016),
        origin=Origin(xyz=(0.140, 0.0, -0.050)),
        material=body_paint,
        name="planetary_housing",
    )
    head.visual(
        Box((0.082, 0.018, 0.022)),
        origin=Origin(xyz=(0.116, -0.028, -0.064)),
        material=body_paint,
        name="underside_left_cheek",
    )
    head.visual(
        Box((0.082, 0.018, 0.022)),
        origin=Origin(xyz=(0.116, 0.028, -0.064)),
        material=body_paint,
        name="underside_right_cheek",
    )
    head.visual(
        Box((0.028, 0.086, 0.010)),
        origin=Origin(xyz=(0.014, 0.0, -0.005)),
        material=body_paint,
        name="rear_support_pad",
    )
    head.inertial = Inertial.from_geometry(
        Box((0.36, 0.17, 0.17)),
        mass=6.0,
        origin=Origin(xyz=(0.17, 0.0, 0.0)),
    )

    tool_shaft = model.part("tool_shaft")
    tool_shaft.visual(
        Cylinder(radius=0.014, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, -0.009)),
        material=stainless,
        name="drive_ferrule",
    )
    tool_shaft.visual(
        Cylinder(radius=0.007, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, -0.048)),
        material=stainless,
        name="shaft_spindle",
    )
    tool_shaft.visual(
        _save_mesh(
            "mixer_dough_hook",
            tube_from_spline_points(
                [
                    (0.000, 0.000, -0.056),
                    (0.009, 0.000, -0.078),
                    (0.022, 0.010, -0.103),
                    (0.030, 0.024, -0.124),
                    (0.022, 0.040, -0.145),
                    (0.002, 0.040, -0.158),
                    (-0.010, 0.024, -0.146),
                    (-0.006, 0.008, -0.126),
                ],
                radius=0.0058,
                samples_per_segment=18,
                radial_segments=18,
                up_hint=(0.0, 0.0, 1.0),
            ),
        ),
        material=stainless,
        name="hook_body",
    )
    tool_shaft.inertial = Inertial.from_geometry(
        Cylinder(radius=0.055, length=0.19),
        mass=0.35,
        origin=Origin(xyz=(0.0, 0.0, -0.095)),
    )

    speed_control = model.part("speed_control")
    speed_control.visual(
        Cylinder(radius=0.018, length=0.016),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim_dark,
        name="knob_body",
    )
    speed_control.visual(
        Box((0.022, 0.010, 0.005)),
        origin=Origin(xyz=(0.013, 0.0, 0.012)),
        material=stainless,
        name="knob_pointer",
    )
    speed_control.inertial = Inertial.from_geometry(
        Box((0.040, 0.020, 0.032)),
        mass=0.08,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    head_lock = model.part("head_lock")
    head_lock.visual(
        Box((0.026, 0.014, 0.006)),
        origin=Origin(xyz=(0.013, 0.0, 0.003)),
        material=trim_dark,
        name="slider_tongue",
    )
    head_lock.visual(
        Box((0.016, 0.020, 0.008)),
        origin=Origin(xyz=(0.019, 0.0, 0.010)),
        material=stainless,
        name="slider_tab",
    )
    head_lock.inertial = Inertial.from_geometry(
        Box((0.028, 0.022, 0.014)),
        mass=0.05,
        origin=Origin(xyz=(0.014, 0.0, 0.007)),
    )

    model.articulation(
        "base_to_carriage",
        ArticulationType.PRISMATIC,
        parent=base,
        child=carriage,
        origin=Origin(xyz=(0.025, 0.0, 0.054)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=0.12, lower=0.0, upper=0.085),
    )
    model.articulation(
        "carriage_to_bowl",
        ArticulationType.FIXED,
        parent=carriage,
        child=bowl,
        origin=Origin(xyz=(0.110, 0.0, 0.042)),
    )
    model.articulation(
        "base_to_head",
        ArticulationType.REVOLUTE,
        parent=base,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, 0.334)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.1,
            lower=0.0,
            upper=math.radians(58.0),
        ),
    )
    model.articulation(
        "head_to_tool_shaft",
        ArticulationType.CONTINUOUS,
        parent=head,
        child=tool_shaft,
        origin=Origin(xyz=(0.140, 0.0, -0.058)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=18.0),
    )
    model.articulation(
        "base_to_speed_control",
        ArticulationType.REVOLUTE,
        parent=base,
        child=speed_control,
        origin=Origin(xyz=(0.008, 0.094, 0.118)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.5,
            velocity=2.0,
            lower=-0.75,
            upper=0.75,
        ),
    )
    model.articulation(
        "base_to_head_lock",
        ArticulationType.PRISMATIC,
        parent=base,
        child=head_lock,
        origin=Origin(xyz=(-0.028, 0.079, 0.048)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=0.08,
            lower=0.0,
            upper=0.014,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    carriage = object_model.get_part("carriage")
    bowl = object_model.get_part("bowl")
    head = object_model.get_part("head")
    tool_shaft = object_model.get_part("tool_shaft")
    speed_control = object_model.get_part("speed_control")
    head_lock = object_model.get_part("head_lock")

    carriage_slide = object_model.get_articulation("base_to_carriage")
    head_hinge = object_model.get_articulation("base_to_head")
    shaft_spin = object_model.get_articulation("head_to_tool_shaft")
    speed_joint = object_model.get_articulation("base_to_speed_control")
    lock_slide = object_model.get_articulation("base_to_head_lock")

    def aabb_center(aabb):
        if aabb is None:
            return None
        return tuple((aabb[0][i] + aabb[1][i]) * 0.5 for i in range(3))

    ctx.expect_contact(
        head,
        tool_shaft,
        elem_a="planetary_housing",
        elem_b="drive_ferrule",
        name="planetary housing supports the rotating shaft",
    )
    ctx.expect_contact(
        base,
        base,
        elem_a="speed_control_bridge",
        elem_b="speed_control_pod",
        name="speed control pod is braced by the side bridge",
    )

    rest_carriage_pos = ctx.part_world_position(carriage)
    rest_bowl_pos = ctx.part_world_position(bowl)
    rest_tool_pos = ctx.part_world_position(tool_shaft)
    rest_lock_pos = ctx.part_world_position(head_lock)
    rest_pointer = aabb_center(ctx.part_element_world_aabb(speed_control, elem="knob_pointer"))
    rest_hook = aabb_center(ctx.part_element_world_aabb(tool_shaft, elem="hook_body"))

    with ctx.pose({carriage_slide: 0.085}):
        extended_carriage_pos = ctx.part_world_position(carriage)
        extended_bowl_pos = ctx.part_world_position(bowl)
        ctx.check(
            "bowl carriage slides forward from the base",
            rest_carriage_pos is not None
            and extended_carriage_pos is not None
            and extended_carriage_pos[0] > rest_carriage_pos[0] + 0.07,
            details=f"rest={rest_carriage_pos}, extended={extended_carriage_pos}",
        )
        ctx.check(
            "bowl moves forward with the carriage",
            rest_bowl_pos is not None
            and extended_bowl_pos is not None
            and extended_bowl_pos[0] > rest_bowl_pos[0] + 0.07,
            details=f"rest={rest_bowl_pos}, extended={extended_bowl_pos}",
        )

    with ctx.pose({head_hinge: math.radians(52.0)}):
        raised_tool_pos = ctx.part_world_position(tool_shaft)
        ctx.check(
            "head hinge lifts the tool upward",
            rest_tool_pos is not None
            and raised_tool_pos is not None
            and raised_tool_pos[2] > rest_tool_pos[2] + 0.07,
            details=f"rest={rest_tool_pos}, raised={raised_tool_pos}",
        )

    with ctx.pose({speed_joint: 0.55}):
        rotated_pointer = aabb_center(ctx.part_element_world_aabb(speed_control, elem="knob_pointer"))
        ctx.check(
            "speed control rotates its pointer",
            rest_pointer is not None
            and rotated_pointer is not None
            and abs(rotated_pointer[2] - rest_pointer[2]) > 0.004,
            details=f"rest={rest_pointer}, rotated={rotated_pointer}",
        )

    with ctx.pose({lock_slide: 0.012}):
        extended_lock_pos = ctx.part_world_position(head_lock)
        ctx.check(
            "head lock control slides outward",
            rest_lock_pos is not None
            and extended_lock_pos is not None
            and extended_lock_pos[0] > rest_lock_pos[0] + 0.009,
            details=f"rest={rest_lock_pos}, extended={extended_lock_pos}",
        )

    with ctx.pose({shaft_spin: math.pi / 2.0}):
        spun_hook = aabb_center(ctx.part_element_world_aabb(tool_shaft, elem="hook_body"))
        ctx.check(
            "dough hook rotates around the vertical mixing axis",
            rest_hook is not None
            and spun_hook is not None
            and (
                abs(spun_hook[0] - rest_hook[0]) > 0.004
                or abs(spun_hook[1] - rest_hook[1]) > 0.004
            ),
            details=f"rest={rest_hook}, spun={spun_hook}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
