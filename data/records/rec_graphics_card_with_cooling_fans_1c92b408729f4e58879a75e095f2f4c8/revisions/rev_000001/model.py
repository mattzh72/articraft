from __future__ import annotations

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BlowerWheelGeometry,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


CARD_LENGTH = 0.320
CARD_WIDTH = 0.120
ROTOR_RADIUS = 0.025
ROTOR_WIDTH = 0.012
HOUSING_INNER_RADIUS = 0.030
HOUSING_OUTER_RADIUS = 0.036
HOUSING_HEIGHT = 0.020
HOUSING_CENTERS = ((0.092, -0.030), (0.092, 0.030))
ROTOR_Z = 0.043


def _annular_housing() -> object:
    """A low, circular blower housing wall with a clear central inlet."""
    return (
        cq.Workplane("XY")
        .circle(HOUSING_OUTER_RADIUS)
        .circle(HOUSING_INNER_RADIUS)
        .extrude(HOUSING_HEIGHT)
        .translate((0.0, 0.0, -HOUSING_HEIGHT / 2.0))
    )


def _rounded_shroud(length: float, width: float, height: float) -> object:
    return (
        cq.Workplane("XY")
        .box(length, width, height)
        .edges("|Z")
        .fillet(0.004)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dual_blower_workstation_graphics_card")

    pcb_green = model.material("solder_mask_green", rgba=(0.02, 0.20, 0.10, 1.0))
    pcb_edge = model.material("gold_contacts", rgba=(0.95, 0.68, 0.18, 1.0))
    aluminum = model.material("brushed_aluminum", rgba=(0.70, 0.72, 0.70, 1.0))
    dark_aluminum = model.material("dark_heatsink", rgba=(0.20, 0.21, 0.22, 1.0))
    shroud_black = model.material("matte_black_shroud", rgba=(0.015, 0.016, 0.018, 1.0))
    satin_gray = model.material("satin_graphite", rgba=(0.18, 0.19, 0.21, 1.0))
    port_black = model.material("black_port_inserts", rgba=(0.0, 0.0, 0.0, 1.0))

    body = model.part("body")
    body.visual(
        Box((CARD_LENGTH, CARD_WIDTH, 0.003)),
        origin=Origin(xyz=(0.0, 0.0, 0.0015)),
        material=pcb_green,
        name="pcb",
    )
    body.visual(
        Box((0.210, 0.006, 0.002)),
        origin=Origin(xyz=(0.030, -0.062, 0.0032)),
        material=pcb_edge,
        name="pcie_edge",
    )
    body.visual(
        Box((0.262, 0.090, 0.012)),
        origin=Origin(xyz=(-0.020, 0.0, 0.0090)),
        material=dark_aluminum,
        name="heatsink_base",
    )

    # Long fins run toward the exhaust bracket under the covered shroud.
    for i, y in enumerate((-0.036, -0.027, -0.018, -0.009, 0.0, 0.009, 0.018, 0.027, 0.036)):
        body.visual(
            Box((0.240, 0.0026, 0.022)),
            origin=Origin(xyz=(-0.030, y, 0.026)),
            material=aluminum,
            name=f"fin_{i}",
        )

    body.visual(
        mesh_from_cadquery(_rounded_shroud(0.225, 0.104, 0.016), "covered_heatsink_tunnel"),
        origin=Origin(xyz=(-0.045, 0.0, 0.039)),
        material=shroud_black,
        name="covered_tunnel",
    )
    body.visual(
        Box((0.260, 0.009, 0.032)),
        origin=Origin(xyz=(-0.010, -0.057, 0.027)),
        material=shroud_black,
        name="lower_side_rail",
    )
    body.visual(
        Box((0.260, 0.009, 0.032)),
        origin=Origin(xyz=(-0.010, 0.057, 0.027)),
        material=shroud_black,
        name="upper_side_rail",
    )

    # Two separate centrifugal scroll throats feed the same fin tunnel and aim left,
    # toward the metal bracket/exhaust end of the workstation card.
    for i, (x, y) in enumerate(HOUSING_CENTERS):
        body.visual(
            mesh_from_cadquery(_annular_housing(), f"blower_housing_{i}"),
            origin=Origin(xyz=(x, y, ROTOR_Z)),
            material=shroud_black,
            name=f"housing_{i}",
        )
        body.visual(
            Box((0.170, 0.026, 0.017)),
            origin=Origin(xyz=(-0.026, y, 0.040)),
            material=satin_gray,
            name=f"exhaust_duct_{i}",
        )
        body.visual(
            Box((0.012, 0.010, 0.016)),
            origin=Origin(xyz=(x - 0.036, y, 0.040)),
            material=satin_gray,
            name=f"volute_tongue_{i}",
        )

    body.visual(
        Box((0.007, 0.144, 0.074)),
        origin=Origin(xyz=(-0.1635, 0.0, 0.037)),
        material=aluminum,
        name="slot_bracket",
    )
    body.visual(
        Box((0.003, 0.036, 0.010)),
        origin=Origin(xyz=(-0.1685, -0.032, 0.042)),
        material=port_black,
        name="display_port_0",
    )
    body.visual(
        Box((0.003, 0.036, 0.010)),
        origin=Origin(xyz=(-0.1685, 0.032, 0.042)),
        material=port_black,
        name="display_port_1",
    )
    body.visual(
        Box((0.005, 0.090, 0.026)),
        origin=Origin(xyz=(-0.166, 0.0, 0.016)),
        material=port_black,
        name="exhaust_grille",
    )

    blower_geometry = BlowerWheelGeometry(
        ROTOR_RADIUS,
        0.012,
        ROTOR_WIDTH,
        24,
        blade_thickness=0.0015,
        blade_sweep_deg=38.0,
        backplate=True,
        shroud=True,
    )
    for i, (x, y) in enumerate(HOUSING_CENTERS):
        rotor = model.part(f"rotor_{i}")
        rotor.visual(
            mesh_from_geometry(blower_geometry, f"blower_wheel_{i}"),
            material=satin_gray,
            name="wheel",
        )
        rotor.visual(
            Cylinder(radius=0.0135, length=ROTOR_WIDTH),
            material=dark_aluminum,
            name="hub",
        )
        rotor.inertial = Inertial.from_geometry(
            Box((ROTOR_RADIUS * 2.0, ROTOR_RADIUS * 2.0, ROTOR_WIDTH)),
            mass=0.035,
        )
        model.articulation(
            f"body_to_rotor_{i}",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=rotor,
            origin=Origin(xyz=(x, y, ROTOR_Z)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=0.15, velocity=450.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")

    for i in range(2):
        rotor = object_model.get_part(f"rotor_{i}")
        joint = object_model.get_articulation(f"body_to_rotor_{i}")
        ctx.check(f"rotor_{i}_exists", rotor is not None, "Expected an articulated blower rotor.")
        ctx.check(
            f"rotor_{i}_continuous_joint",
            joint is not None and str(joint.articulation_type).lower().endswith("continuous"),
            f"joint={joint}",
        )
        if rotor is None or joint is None:
            continue

        ctx.check(
            f"rotor_{i}_axial_axis",
            tuple(round(v, 4) for v in joint.axis) == (0.0, 0.0, 1.0),
            f"axis={joint.axis!r}",
        )
        ctx.expect_within(
            rotor,
            body,
            axes="xy",
            inner_elem="wheel",
            outer_elem=f"housing_{i}",
            margin=0.002,
            name=f"rotor_{i}_within_circular_housing",
        )
        ctx.expect_overlap(
            rotor,
            body,
            axes="z",
            elem_a="wheel",
            elem_b=f"housing_{i}",
            min_overlap=ROTOR_WIDTH * 0.75,
            name=f"rotor_{i}_sits_inside_housing_depth",
        )

        rest_pos = ctx.part_world_position(rotor)
        with ctx.pose({joint: 1.25}):
            spun_pos = ctx.part_world_position(rotor)
        ctx.check(
            f"rotor_{i}_spins_about_fixed_center",
            rest_pos is not None
            and spun_pos is not None
            and all(abs(rest_pos[k] - spun_pos[k]) < 1e-6 for k in range(3)),
            details=f"rest={rest_pos}, spun={spun_pos}",
        )

    duct_0 = ctx.part_element_world_aabb(body, elem="exhaust_duct_0")
    bracket = ctx.part_element_world_aabb(body, elem="slot_bracket")
    housing_0 = ctx.part_element_world_aabb(body, elem="housing_0")
    ctx.check(
        "ducts_run_toward_bracket",
        duct_0 is not None
        and bracket is not None
        and housing_0 is not None
        and duct_0[0][0] < housing_0[0][0]
        and bracket[0][0] < duct_0[0][0],
        details=f"duct={duct_0}, bracket={bracket}, housing={housing_0}",
    )

    return ctx.report()


object_model = build_object_model()
