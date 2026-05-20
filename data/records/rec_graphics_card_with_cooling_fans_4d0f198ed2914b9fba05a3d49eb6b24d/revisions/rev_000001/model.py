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
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


CARD_LENGTH = 0.274
CARD_HEIGHT = 0.106
CARD_THICKNESS = 0.036

BLOWER_OUTER_RADIUS = 0.024
BLOWER_INNER_RADIUS = 0.018
BLOWER_HOUSING_DEPTH = 0.012
BLOWER_ROTOR_OUTER_RADIUS = 0.0165
BLOWER_ROTOR_RIM_INNER_RADIUS = 0.013
BLOWER_ROTOR_DEPTH = 0.007
MOTOR_POD_RADIUS = 0.0055
MOTOR_POD_LENGTH = 0.006

LEFT_BLOWER_CENTER = (-0.088, 0.024, 0.028)
RIGHT_BLOWER_CENTER = (-0.038, 0.024, 0.028)


def _annular_shell_mesh(
    *,
    outer_radius: float,
    inner_radius: float,
    depth: float,
    name: str,
):
    geometry = LatheGeometry.from_shell_profiles(
        [(outer_radius, -depth / 2.0), (outer_radius, depth / 2.0)],
        [(inner_radius, -depth / 2.0), (inner_radius, depth / 2.0)],
        segments=56,
        start_cap="flat",
        end_cap="flat",
    )
    return mesh_from_geometry(geometry, name)


def _add_blower_rotor(
    part,
    *,
    mesh_name: str,
    rotor_disc_name: str,
    hub_name: str,
    blade_prefix: str,
    body_color,
    rotor_color,
) -> None:
    rim_mesh = _annular_shell_mesh(
        outer_radius=BLOWER_ROTOR_OUTER_RADIUS,
        inner_radius=BLOWER_ROTOR_RIM_INNER_RADIUS,
        depth=BLOWER_ROTOR_DEPTH,
        name=mesh_name,
    )
    part.visual(
        rim_mesh,
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=body_color,
        name=rotor_disc_name,
    )
    part.visual(
        Cylinder(radius=0.0055, length=BLOWER_ROTOR_DEPTH + 0.001),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=rotor_color,
        name=hub_name,
    )

    blade_count = 10
    blade_radius = 0.010
    for blade_index in range(blade_count):
        angle = 2.0 * math.pi * blade_index / blade_count
        part.visual(
            Box((0.009, BLOWER_ROTOR_DEPTH * 0.9, 0.0014)),
            origin=Origin(
                xyz=(
                    blade_radius * math.cos(angle),
                    0.0,
                    blade_radius * math.sin(angle),
                ),
                rpy=(0.0, angle, 0.0),
            ),
            material=rotor_color,
            name=f"{blade_prefix}_{blade_index}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="workstation_graphics_card")

    shroud_black = model.material("shroud_black", rgba=(0.11, 0.12, 0.13, 1.0))
    housing_black = model.material("housing_black", rgba=(0.07, 0.08, 0.09, 1.0))
    pcb_green = model.material("pcb_green", rgba=(0.16, 0.22, 0.12, 1.0))
    bracket_metal = model.material("bracket_metal", rgba=(0.72, 0.75, 0.79, 1.0))
    connector_gold = model.material("connector_gold", rgba=(0.79, 0.63, 0.18, 1.0))
    rotor_gray = model.material("rotor_gray", rgba=(0.40, 0.42, 0.46, 1.0))
    fin_metal = model.material("fin_metal", rgba=(0.56, 0.59, 0.63, 1.0))

    card_body = model.part("card_body")
    card_body.visual(
        Box((CARD_LENGTH, CARD_THICKNESS, CARD_HEIGHT)),
        material=shroud_black,
        name="main_shell",
    )
    card_body.visual(
        Box((CARD_LENGTH + 0.006, 0.003, CARD_HEIGHT - 0.010)),
        origin=Origin(xyz=(0.0, -(CARD_THICKNESS / 2.0 + 0.0015), -0.001)),
        material=pcb_green,
        name="pcb_board",
    )
    card_body.visual(
        Box((0.012, 0.003, CARD_HEIGHT + 0.006)),
        origin=Origin(xyz=(-(CARD_LENGTH / 2.0 + 0.006), 0.0, 0.0)),
        material=bracket_metal,
        name="io_bracket",
    )
    card_body.visual(
        Box((0.082, 0.006, 0.008)),
        origin=Origin(xyz=(0.038, -CARD_THICKNESS / 2.0, -(CARD_HEIGHT / 2.0 + 0.001))),
        material=connector_gold,
        name="pcie_connector",
    )
    card_body.visual(
        Box((0.026, CARD_THICKNESS - 0.004, 0.078)),
        origin=Origin(xyz=((CARD_LENGTH / 2.0) - 0.018, 0.0, -0.006)),
        material=fin_metal,
        name="front_fin_stack",
    )
    card_body.visual(
        Box((0.092, 0.010, 0.018)),
        origin=Origin(xyz=(-0.062, 0.013, CARD_HEIGHT / 2.0 - 0.015)),
        material=housing_black,
        name="top_blower_rail",
    )

    left_housing_mesh = _annular_shell_mesh(
        outer_radius=BLOWER_OUTER_RADIUS,
        inner_radius=BLOWER_INNER_RADIUS,
        depth=BLOWER_HOUSING_DEPTH,
        name="left_blower_housing_shell",
    )
    right_housing_mesh = _annular_shell_mesh(
        outer_radius=BLOWER_OUTER_RADIUS,
        inner_radius=BLOWER_INNER_RADIUS,
        depth=BLOWER_HOUSING_DEPTH,
        name="right_blower_housing_shell",
    )
    card_body.visual(
        left_housing_mesh,
        origin=Origin(xyz=LEFT_BLOWER_CENTER, rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=housing_black,
        name="left_blower_housing",
    )
    card_body.visual(
        right_housing_mesh,
        origin=Origin(xyz=RIGHT_BLOWER_CENTER, rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=housing_black,
        name="right_blower_housing",
    )
    card_body.visual(
        Cylinder(radius=MOTOR_POD_RADIUS, length=MOTOR_POD_LENGTH),
        origin=Origin(
            xyz=(LEFT_BLOWER_CENTER[0], LEFT_BLOWER_CENTER[1] - 0.007, LEFT_BLOWER_CENTER[2]),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        material=fin_metal,
        name="left_motor_pod",
    )
    card_body.visual(
        Cylinder(radius=MOTOR_POD_RADIUS, length=MOTOR_POD_LENGTH),
        origin=Origin(
            xyz=(RIGHT_BLOWER_CENTER[0], RIGHT_BLOWER_CENTER[1] - 0.007, RIGHT_BLOWER_CENTER[2]),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        material=fin_metal,
        name="right_motor_pod",
    )

    left_blower_rotor = model.part("left_blower_rotor")
    _add_blower_rotor(
        left_blower_rotor,
        mesh_name="left_rotor_rim",
        rotor_disc_name="left_rotor_disc",
        hub_name="left_hub",
        blade_prefix="left_blade",
        body_color=housing_black,
        rotor_color=rotor_gray,
    )

    right_blower_rotor = model.part("right_blower_rotor")
    _add_blower_rotor(
        right_blower_rotor,
        mesh_name="right_rotor_rim",
        rotor_disc_name="right_rotor_disc",
        hub_name="right_hub",
        blade_prefix="right_blade",
        body_color=housing_black,
        rotor_color=rotor_gray,
    )

    model.articulation(
        "left_blower_spin",
        ArticulationType.CONTINUOUS,
        parent=card_body,
        child=left_blower_rotor,
        origin=Origin(xyz=LEFT_BLOWER_CENTER),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.35, velocity=260.0),
    )
    model.articulation(
        "right_blower_spin",
        ArticulationType.CONTINUOUS,
        parent=card_body,
        child=right_blower_rotor,
        origin=Origin(xyz=RIGHT_BLOWER_CENTER),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.35, velocity=260.0),
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

    card_body = object_model.get_part("card_body")
    left_blower_rotor = object_model.get_part("left_blower_rotor")
    right_blower_rotor = object_model.get_part("right_blower_rotor")
    left_blower_spin = object_model.get_articulation("left_blower_spin")
    right_blower_spin = object_model.get_articulation("right_blower_spin")

    left_limits = left_blower_spin.motion_limits
    right_limits = right_blower_spin.motion_limits

    ctx.check(
        "left blower uses a continuous axial joint",
        left_blower_spin.articulation_type == ArticulationType.CONTINUOUS
        and tuple(left_blower_spin.axis) == (0.0, 1.0, 0.0)
        and left_limits is not None
        and left_limits.lower is None
        and left_limits.upper is None,
        details=(
            f"type={left_blower_spin.articulation_type}, "
            f"axis={left_blower_spin.axis}, limits={left_limits}"
        ),
    )
    ctx.check(
        "right blower uses a continuous axial joint",
        right_blower_spin.articulation_type == ArticulationType.CONTINUOUS
        and tuple(right_blower_spin.axis) == (0.0, 1.0, 0.0)
        and right_limits is not None
        and right_limits.lower is None
        and right_limits.upper is None,
        details=(
            f"type={right_blower_spin.articulation_type}, "
            f"axis={right_blower_spin.axis}, limits={right_limits}"
        ),
    )

    with ctx.pose({left_blower_spin: 0.0, right_blower_spin: 0.0}):
        ctx.expect_contact(
            left_blower_rotor,
            card_body,
            elem_a="left_hub",
            elem_b="left_motor_pod",
            contact_tol=1e-6,
            name="left rotor is supported by the left motor pod",
        )
        ctx.expect_contact(
            right_blower_rotor,
            card_body,
            elem_a="right_hub",
            elem_b="right_motor_pod",
            contact_tol=1e-6,
            name="right rotor is supported by the right motor pod",
        )
        ctx.expect_within(
            left_blower_rotor,
            card_body,
            axes="xz",
            inner_elem="left_rotor_disc",
            outer_elem="left_blower_housing",
            margin=0.0025,
            name="left rotor fits within the left blower housing opening",
        )
        ctx.expect_within(
            right_blower_rotor,
            card_body,
            axes="xz",
            inner_elem="right_rotor_disc",
            outer_elem="right_blower_housing",
            margin=0.0025,
            name="right rotor fits within the right blower housing opening",
        )
        ctx.expect_gap(
            left_blower_rotor,
            card_body,
            axis="y",
            min_gap=0.001,
            max_gap=0.004,
            positive_elem="left_rotor_disc",
            negative_elem="main_shell",
            name="left rotor stands proud of the shroud face",
        )
        ctx.expect_gap(
            right_blower_rotor,
            card_body,
            axis="y",
            min_gap=0.001,
            max_gap=0.004,
            positive_elem="right_rotor_disc",
            negative_elem="main_shell",
            name="right rotor stands proud of the shroud face",
        )

    with ctx.pose({left_blower_spin: 2.2, right_blower_spin: -1.7}):
        ctx.expect_within(
            left_blower_rotor,
            card_body,
            axes="xz",
            inner_elem="left_rotor_disc",
            outer_elem="left_blower_housing",
            margin=0.0025,
            name="left rotor stays centered while spinning",
        )
        ctx.expect_within(
            right_blower_rotor,
            card_body,
            axes="xz",
            inner_elem="right_rotor_disc",
            outer_elem="right_blower_housing",
            margin=0.0025,
            name="right rotor stays centered while spinning",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
