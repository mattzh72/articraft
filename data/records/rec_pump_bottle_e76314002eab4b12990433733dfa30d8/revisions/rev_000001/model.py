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
    DomeGeometry,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="foaming_soap_pump_bottle")

    bottle_clear = model.material("bottle_clear", rgba=(0.76, 0.92, 0.88, 0.58))
    pump_white = model.material("pump_white", rgba=(0.95, 0.96, 0.97, 1.0))
    pump_light = model.material("pump_light", rgba=(0.90, 0.91, 0.92, 1.0))

    body_outer = 0.082
    body_inner = 0.074
    body_height = 0.145
    body_corner = 0.010
    top_plate_thickness = 0.004
    neck_height = 0.018
    collar_height = 0.014
    collar_top_z = body_height + top_plate_thickness + neck_height + collar_height

    bottle = model.part("bottle")
    bottle.inertial = Inertial.from_geometry(
        Box((0.085, 0.085, collar_top_z)),
        mass=0.46,
        origin=Origin(xyz=(0.0, 0.0, collar_top_z * 0.5)),
    )

    bottle.visual(
        mesh_from_geometry(
            ExtrudeWithHolesGeometry(
                rounded_rect_profile(body_outer, body_outer, body_corner, corner_segments=10),
                [rounded_rect_profile(body_inner, body_inner, max(body_corner - 0.003, 0.003), corner_segments=10)],
                height=body_height,
                center=False,
            ),
            "bottle_body_shell",
        ),
        material=bottle_clear,
        name="body_shell",
    )
    bottle.visual(
        Box((body_inner, body_inner, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=bottle_clear,
        name="bottom_panel",
    )
    bottle.visual(
        mesh_from_geometry(
            ExtrudeWithHolesGeometry(
                rounded_rect_profile(body_outer, body_outer, body_corner, corner_segments=10),
                [_circle_profile(0.0124, segments=36)],
                height=top_plate_thickness,
                center=False,
            ),
            "bottle_top_cap",
        ),
        origin=Origin(xyz=(0.0, 0.0, body_height)),
        material=bottle_clear,
        name="top_cap",
    )
    bottle.visual(
        mesh_from_geometry(
            ExtrudeWithHolesGeometry(
                _circle_profile(0.0160, segments=36),
                [_circle_profile(0.0113, segments=36)],
                height=neck_height,
                center=False,
            ),
            "bottle_neck_sleeve",
        ),
        origin=Origin(xyz=(0.0, 0.0, body_height + top_plate_thickness)),
        material=bottle_clear,
        name="neck_sleeve",
    )
    bottle.visual(
        mesh_from_geometry(
            ExtrudeWithHolesGeometry(
                _circle_profile(0.0220, segments=40),
                [_circle_profile(0.0128, segments=40)],
                height=collar_height,
                center=False,
            ),
            "bottle_pump_collar",
        ),
        origin=Origin(xyz=(0.0, 0.0, body_height + top_plate_thickness + neck_height)),
        material=pump_white,
        name="pump_collar",
    )

    plunger = model.part("plunger")
    plunger.inertial = Inertial.from_geometry(
        Box((0.070, 0.050, 0.120)),
        mass=0.055,
        origin=Origin(xyz=(0.0, 0.0, -0.010)),
    )
    plunger.visual(
        mesh_from_geometry(
            ExtrudeGeometry(
                _circle_profile(0.0113, segments=36),
                0.096,
                center=True,
            ),
            "plunger_stem",
        ),
        origin=Origin(xyz=(0.0, 0.0, -0.028)),
        material=pump_light,
        name="stem",
    )
    plunger.visual(
        Cylinder(radius=0.0165, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=pump_white,
        name="actuator_skirt",
    )
    plunger.visual(
        Cylinder(radius=0.031, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.022)),
        material=pump_white,
        name="head_plate",
    )
    plunger.visual(
        mesh_from_geometry(
            DomeGeometry(radius=0.031, radial_segments=40, height_segments=16).scale(1.0, 1.0, 0.38),
            "plunger_head_dome",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.026)),
        material=pump_white,
        name="head_dome",
    )
    plunger.visual(
        Cylinder(radius=0.0085, length=0.024),
        origin=Origin(xyz=(0.022, 0.0, 0.020), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=pump_white,
        name="nozzle_mount",
    )
    plunger.visual(
        Cylinder(radius=0.0024, length=0.016),
        origin=Origin(xyz=(0.030, 0.0, 0.027), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=pump_white,
        name="pivot_bridge",
    )
    plunger.visual(
        mesh_from_geometry(
            ExtrudeGeometry(
                _circle_profile(0.0032, segments=32),
                0.020,
                center=True,
            ),
            "plunger_pivot_pin",
        ),
        origin=Origin(xyz=(0.038, 0.0, 0.0195)),
        material=pump_light,
        name="pivot_pin",
    )

    nozzle = model.part("nozzle")
    nozzle.inertial = Inertial.from_geometry(
        Box((0.040, 0.016, 0.018)),
        mass=0.012,
        origin=Origin(xyz=(0.018, 0.0, 0.0)),
    )
    nozzle.visual(
        mesh_from_geometry(
            ExtrudeWithHolesGeometry(
                _circle_profile(0.0054, segments=32),
                [_circle_profile(0.0032, segments=32)],
                height=0.009,
                center=False,
            ),
            "nozzle_pivot_ring",
        ),
        origin=Origin(xyz=(0.0, 0.0, -0.0045)),
        material=pump_light,
        name="pivot_ring",
    )
    nozzle.visual(
        Cylinder(radius=0.0046, length=0.020),
        origin=Origin(xyz=(0.014, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=pump_white,
        name="spout_body",
    )
    nozzle.visual(
        Cylinder(radius=0.0036, length=0.012),
        origin=Origin(xyz=(0.029, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=pump_white,
        name="spout_tip",
    )

    model.articulation(
        "bottle_to_plunger",
        ArticulationType.PRISMATIC,
        parent=bottle,
        child=plunger,
        origin=Origin(xyz=(0.0, 0.0, collar_top_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=0.08,
            lower=-0.006,
            upper=0.0,
        ),
    )
    model.articulation(
        "plunger_to_nozzle",
        ArticulationType.REVOLUTE,
        parent=plunger,
        child=nozzle,
        origin=Origin(xyz=(0.038, 0.0, 0.0195)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=2.0,
            lower=-1.2,
            upper=1.2,
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
    bottle = object_model.get_part("bottle")
    plunger = object_model.get_part("plunger")
    nozzle = object_model.get_part("nozzle")
    plunger_slide = object_model.get_articulation("bottle_to_plunger")
    nozzle_pivot = object_model.get_articulation("plunger_to_nozzle")

    ctx.allow_overlap(
        bottle,
        plunger,
        elem_a="neck_sleeve",
        elem_b="stem",
        reason="The pump stem is intentionally modeled as guided inside the neck sleeve proxy.",
    )
    ctx.allow_overlap(
        plunger,
        nozzle,
        elem_a="pivot_pin",
        elem_b="pivot_ring",
        reason="The nozzle barrel is intentionally represented as rotating around the pump-head pivot pin.",
    )

    ctx.expect_within(
        plunger,
        bottle,
        axes="xy",
        inner_elem="stem",
        outer_elem="neck_sleeve",
        margin=0.0065,
        name="plunger stem stays centered in the neck sleeve",
    )
    ctx.expect_overlap(
        plunger,
        bottle,
        axes="z",
        elem_a="stem",
        elem_b="neck_sleeve",
        min_overlap=0.015,
        name="plunger stem remains inserted in the neck",
    )
    ctx.expect_gap(
        plunger,
        bottle,
        axis="z",
        positive_elem="actuator_skirt",
        negative_elem="pump_collar",
        min_gap=0.004,
        max_gap=0.007,
        name="resting plunger head sits slightly above the collar",
    )

    rest_plunger_pos = ctx.part_world_position(plunger)
    rest_tip_aabb = ctx.part_element_world_aabb(nozzle, elem="spout_tip")
    with ctx.pose({plunger_slide: -0.006}):
        pressed_plunger_pos = ctx.part_world_position(plunger)
        ctx.expect_gap(
            plunger,
            bottle,
            axis="z",
            positive_elem="actuator_skirt",
            negative_elem="pump_collar",
            max_gap=0.001,
            max_penetration=0.0,
            name="pressed plunger comes down to the collar seat",
        )
        ctx.expect_overlap(
            plunger,
            bottle,
            axes="z",
            elem_a="stem",
            elem_b="neck_sleeve",
            min_overlap=0.015,
            name="pressed plunger still retains stem insertion",
        )

    ctx.check(
        "plunger translates downward when pressed",
        rest_plunger_pos is not None
        and pressed_plunger_pos is not None
        and pressed_plunger_pos[2] < rest_plunger_pos[2] - 0.004,
        details=f"rest={rest_plunger_pos}, pressed={pressed_plunger_pos}",
    )

    with ctx.pose({nozzle_pivot: 0.9}):
        turned_tip_aabb = ctx.part_element_world_aabb(nozzle, elem="spout_tip")

    ctx.check(
        "nozzle swings laterally on its pivot",
        rest_tip_aabb is not None
        and turned_tip_aabb is not None
        and ((turned_tip_aabb[0][1] + turned_tip_aabb[1][1]) * 0.5)
        > ((rest_tip_aabb[0][1] + rest_tip_aabb[1][1]) * 0.5) + 0.010,
        details=f"rest={rest_tip_aabb}, turned={turned_tip_aabb}",
    )

    return ctx.report()


def _circle_profile(radius: float, *, segments: int = 32) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos((2.0 * math.pi * index) / segments),
            radius * math.sin((2.0 * math.pi * index) / segments),
        )
        for index in range(segments)
    ]


# >>> USER_CODE_END

object_model = build_object_model()
