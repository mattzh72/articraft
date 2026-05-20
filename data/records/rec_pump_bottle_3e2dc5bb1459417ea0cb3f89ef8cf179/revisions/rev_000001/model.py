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
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="kitchen_pump_bottle")

    bottle_ceramic = model.material("bottle_ceramic", rgba=(0.90, 0.88, 0.84, 1.0))
    pump_black = model.material("pump_black", rgba=(0.14, 0.14, 0.15, 1.0))
    stem_metal = model.material("stem_metal", rgba=(0.73, 0.76, 0.79, 1.0))

    bottle = model.part("bottle")

    bottle.visual(
        mesh_from_geometry(
            LatheGeometry.from_shell_profiles(
                [
                    (0.018, 0.000),
                    (0.033, 0.006),
                    (0.045, 0.024),
                    (0.048, 0.096),
                    (0.047, 0.166),
                    (0.044, 0.196),
                    (0.036, 0.220),
                    (0.023, 0.232),
                ],
                [
                    (0.000, 0.004),
                    (0.026, 0.010),
                    (0.039, 0.024),
                    (0.043, 0.096),
                    (0.042, 0.164),
                    (0.039, 0.192),
                    (0.031, 0.216),
                    (0.016, 0.232),
                ],
                segments=72,
                start_cap="flat",
                end_cap="flat",
            ),
            "bottle_shell",
        ),
        material=bottle_ceramic,
        name="body_shell",
    )
    bottle.visual(
        mesh_from_geometry(
            LatheGeometry.from_shell_profiles(
                [
                    (0.0245, 0.228),
                    (0.0280, 0.232),
                    (0.0280, 0.254),
                    (0.0260, 0.258),
                ],
                [
                    (0.0170, 0.228),
                    (0.0185, 0.232),
                    (0.0185, 0.254),
                    (0.0170, 0.258),
                ],
                segments=64,
                start_cap="flat",
                end_cap="flat",
            ),
            "neck_collar",
        ),
        material=pump_black,
        name="neck_collar",
    )
    bottle.visual(
        Box((0.010, 0.020, 0.030)),
        origin=Origin(xyz=(-0.021, 0.0, 0.243)),
        material=pump_black,
        name="rear_guide",
    )
    bottle.inertial = Inertial.from_geometry(
        Box((0.100, 0.100, 0.258)),
        mass=0.95,
        origin=Origin(xyz=(0.0, 0.0, 0.129)),
    )

    actuator = model.part("actuator")
    actuator.visual(
        Cylinder(radius=0.0045, length=0.118),
        origin=Origin(xyz=(0.0, 0.0, -0.059)),
        material=stem_metal,
        name="lower_stem",
    )
    actuator.visual(
        Cylinder(radius=0.0060, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=stem_metal,
        name="upper_stem",
    )
    actuator.visual(
        Box((0.020, 0.020, 0.024)),
        origin=Origin(xyz=(-0.006, 0.0, 0.012)),
        material=pump_black,
        name="rear_skirt",
    )
    actuator.visual(
        Cylinder(radius=0.011, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        material=pump_black,
        name="actuator_core",
    )
    actuator.visual(
        Box((0.052, 0.032, 0.024)),
        origin=Origin(xyz=(0.010, 0.0, 0.019)),
        material=pump_black,
        name="actuator_block",
    )
    actuator.visual(
        Box((0.024, 0.026, 0.014)),
        origin=Origin(xyz=(0.046, 0.0, 0.031)),
        material=pump_black,
        name="nozzle_mount_nose",
    )
    actuator.visual(
        mesh_from_geometry(
            ExtrudeGeometry(
                rounded_rect_profile(0.122, 0.055, 0.010, corner_segments=8),
                0.010,
                cap=True,
                center=True,
                closed=True,
            ),
            "actuator_plate",
        ),
        origin=Origin(xyz=(0.016, 0.0, 0.034)),
        material=pump_black,
        name="actuator_plate",
    )
    actuator.visual(
        Cylinder(radius=0.008, length=0.008),
        origin=Origin(xyz=(0.050, 0.0, 0.028)),
        material=pump_black,
        name="nozzle_swivel_boss",
    )
    actuator.inertial = Inertial.from_geometry(
        Box((0.128, 0.060, 0.170)),
        mass=0.12,
        origin=Origin(xyz=(0.010, 0.0, -0.025)),
    )

    nozzle = model.part("nozzle")
    nozzle.visual(
        Cylinder(radius=0.0065, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, -0.004)),
        material=pump_black,
        name="nozzle_collar",
    )
    nozzle.visual(
        Cylinder(radius=0.0075, length=0.026),
        origin=Origin(xyz=(0.013, 0.0, -0.004), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=pump_black,
        name="nozzle_barrel",
    )
    nozzle.visual(
        Cylinder(radius=0.0045, length=0.034),
        origin=Origin(xyz=(0.043, 0.0, -0.004), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=pump_black,
        name="nozzle_tip",
    )
    nozzle.visual(
        Cylinder(radius=0.0035, length=0.012),
        origin=Origin(xyz=(0.058, 0.0, -0.010)),
        material=pump_black,
        name="outlet_tip",
    )
    nozzle.inertial = Inertial.from_geometry(
        Box((0.068, 0.020, 0.018)),
        mass=0.03,
        origin=Origin(xyz=(0.032, 0.0, -0.006)),
    )

    model.articulation(
        "bottle_to_actuator",
        ArticulationType.PRISMATIC,
        parent=bottle,
        child=actuator,
        origin=Origin(xyz=(0.0, 0.0, 0.258)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=0.12,
            lower=0.0,
            upper=0.018,
        ),
    )
    model.articulation(
        "actuator_to_nozzle",
        ArticulationType.REVOLUTE,
        parent=actuator,
        child=nozzle,
        origin=Origin(xyz=(0.050, 0.0, 0.024)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.0,
            lower=-1.05,
            upper=1.05,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    bottle = object_model.get_part("bottle")
    actuator = object_model.get_part("actuator")
    nozzle = object_model.get_part("nozzle")
    pump_slide = object_model.get_articulation("bottle_to_actuator")
    nozzle_swivel = object_model.get_articulation("actuator_to_nozzle")

    with ctx.pose({pump_slide: 0.0, nozzle_swivel: 0.0}):
        ctx.expect_within(
            actuator,
            bottle,
            axes="xy",
            inner_elem="lower_stem",
            outer_elem="neck_collar",
            margin=0.0,
            name="pump stem stays centered within the neck collar",
        )
        ctx.expect_gap(
            actuator,
            bottle,
            axis="z",
            positive_elem="actuator_plate",
            negative_elem="neck_collar",
            min_gap=0.015,
            max_gap=0.040,
            name="resting actuator plate sits above the bottle collar",
        )
        ctx.expect_contact(
            actuator,
            bottle,
            elem_a="rear_skirt",
            elem_b="rear_guide",
            contact_tol=1e-6,
            name="actuator rear skirt rides on the bottle guide",
        )
        ctx.expect_contact(
            nozzle,
            actuator,
            elem_a="nozzle_collar",
            elem_b="nozzle_swivel_boss",
            contact_tol=1e-6,
            name="nozzle collar seats against the actuator swivel boss",
        )

    rest_z = None
    pressed_z = None
    with ctx.pose({pump_slide: 0.0}):
        rest_pos = ctx.part_world_position(actuator)
        if rest_pos is not None:
            rest_z = rest_pos[2]
    with ctx.pose({pump_slide: 0.018}):
        ctx.expect_gap(
            actuator,
            bottle,
            axis="z",
            positive_elem="actuator_plate",
            negative_elem="neck_collar",
            min_gap=0.000,
            max_gap=0.024,
            name="pressed actuator still clears the bottle collar",
        )
        ctx.expect_contact(
            actuator,
            bottle,
            elem_a="rear_skirt",
            elem_b="rear_guide",
            contact_tol=1e-6,
            name="rear guide stays engaged through the pump stroke",
        )
        pressed_pos = ctx.part_world_position(actuator)
        if pressed_pos is not None:
            pressed_z = pressed_pos[2]
    ctx.check(
        "actuator translates downward when pressed",
        rest_z is not None and pressed_z is not None and pressed_z < rest_z - 0.010,
        details=f"rest_z={rest_z}, pressed_z={pressed_z}",
    )

    nozzle_rest_aabb = None
    nozzle_turned_aabb = None
    with ctx.pose({nozzle_swivel: 0.0}):
        nozzle_rest_aabb = ctx.part_world_aabb(nozzle)
    with ctx.pose({nozzle_swivel: 0.90}):
        ctx.expect_contact(
            nozzle,
            actuator,
            elem_a="nozzle_collar",
            elem_b="nozzle_swivel_boss",
            contact_tol=1e-6,
            name="nozzle remains seated on its swivel boss while turned",
        )
        nozzle_turned_aabb = ctx.part_world_aabb(nozzle)
    ctx.check(
        "nozzle swivels laterally around its own joint",
        nozzle_rest_aabb is not None
        and nozzle_turned_aabb is not None
        and nozzle_turned_aabb[1][1] > nozzle_rest_aabb[1][1] + 0.020
        and nozzle_turned_aabb[1][0] < nozzle_rest_aabb[1][0] - 0.010,
        details=f"rest_aabb={nozzle_rest_aabb}, turned_aabb={nozzle_turned_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
