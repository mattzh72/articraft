from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    wire_from_points,
)


X_AXIS = (0.0, math.pi / 2.0, 0.0)
Y_AXIS = (-math.pi / 2.0, 0.0, 0.0)


def _rod_x(part, *, x, y, z, length, radius, material, name):
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=(x, y, z), rpy=X_AXIS),
        material=material,
        name=name,
    )


def _rod_y(part, *, x, y, z, length, radius, material, name):
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=(x, y, z), rpy=Y_AXIS),
        material=material,
        name=name,
    )


def _add_wing_geometry(wing, *, sign: float, rail_material, gasket_material, hardware_material):
    """Add one side wing in its local hinge frame.

    The hinge line is the local Y axis at x=0, z=0.  Positive-side wings extend
    in local +X; mirrored wings extend in local -X.
    """

    wing.visual(
        Box((0.075, 1.16, 0.018)),
        origin=Origin(xyz=(sign * 0.075, 0.0, -0.020)),
        material=gasket_material,
        name="hinge_boot",
    )
    wing.visual(
        Box((0.085, 1.16, 0.016)),
        origin=Origin(xyz=(sign * 0.083, 0.0, -0.006)),
        material=hardware_material,
        name="hinge_leaf",
    )

    # Rectangular perimeter: two side rails, a protected outer header, and a
    # short wet-edge lip that throws drips below the laundry rails.
    _rod_x(
        wing,
        x=sign * 0.425,
        y=0.54,
        z=0.0,
        length=0.78,
        radius=0.012,
        material=rail_material,
        name="side_rail_0",
    )
    _rod_x(
        wing,
        x=sign * 0.425,
        y=-0.54,
        z=0.0,
        length=0.78,
        radius=0.012,
        material=rail_material,
        name="side_rail_1",
    )
    _rod_y(
        wing,
        x=sign * 0.80,
        y=0.0,
        z=0.0,
        length=1.12,
        radius=0.014,
        material=rail_material,
        name="outer_header",
    )
    wing.visual(
        Box((0.028, 1.13, 0.014)),
        origin=Origin(xyz=(sign * 0.825, 0.0, -0.018)),
        material=gasket_material,
        name="drip_lip",
    )

    # Weatherproof stop links: welded diagonal bars under the wing carry load
    # from the hinge leaf into the rails and visibly define the deployed stop.
    for idx, y in enumerate((-0.43, 0.43)):
        wing.visual(
            Box((0.37, 0.026, 0.018)),
            origin=Origin(xyz=(sign * 0.225, math.copysign(0.54, y), -0.006)),
            material=hardware_material,
            name=f"stop_link_{idx}",
        )

    for idx, x in enumerate((0.16, 0.28, 0.40, 0.52, 0.64)):
        _rod_y(
            wing,
            x=sign * x,
            y=0.0,
            z=0.009,
            length=1.08,
            radius=0.0065,
            material=rail_material,
            name=f"hanging_rail_{idx}",
        )

    # Small sealed end caps on the rail tips read as plastic/rubber plugs and
    # keep the tube ends from looking open to rain.
    for idx, y in enumerate((-0.54, 0.54)):
        wing.visual(
            Box((0.034, 0.032, 0.026)),
            origin=Origin(xyz=(sign * 0.808, y, 0.0)),
            material=gasket_material,
            name=f"sealed_cap_{idx}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="outdoor_weatherproof_drying_rack")

    powder = model.material("powder_coated_aluminum", rgba=(0.78, 0.83, 0.82, 1.0))
    stainless = model.material("brushed_stainless_hardware", rgba=(0.70, 0.72, 0.70, 1.0))
    dark_gasket = model.material("black_epdm_seals", rgba=(0.02, 0.025, 0.025, 1.0))
    foot_rubber = model.material("dark_anchor_feet", rgba=(0.05, 0.055, 0.055, 1.0))

    base = model.part("base")

    # The central sealed beam is a slightly overbuilt weatherproof spine.  It
    # ties the legs, fixed rails, hinge sills, rain hoods, and ground anchors
    # into one continuous root part.
    base.visual(
        Box((0.54, 1.32, 0.065)),
        origin=Origin(xyz=(0.0, 0.0, 0.838)),
        material=powder,
        name="sealed_spine",
    )
    base.visual(
        Box((0.58, 1.37, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, 0.883)),
        material=powder,
        name="drip_overhang",
    )

    for idx, x in enumerate((-0.145, -0.055, 0.055, 0.145)):
        _rod_y(
            base,
            x=x,
            y=0.0,
            z=0.894,
            length=1.22,
            radius=0.0075,
            material=powder,
            name=f"center_rail_{idx}",
        )
    for idx, y in enumerate((-0.61, 0.61)):
        _rod_x(
            base,
            x=0.0,
            y=y,
            z=0.894,
            length=0.40,
            radius=0.010,
            material=powder,
            name=f"center_header_{idx}",
        )

    # Hinge sills, stainless pins, and sloped rain hoods.  The hoods overhang
    # the pin line so runoff sheds outside the protected joint rather than into
    # the barrels.
    for idx, x in enumerate((-0.255, 0.255)):
        base.visual(
            Box((0.046, 1.31, 0.046)),
            origin=Origin(xyz=(x, 0.0, 0.844)),
            material=powder,
            name=f"hinge_sill_{idx}",
        )
        _rod_y(
            base,
            x=x,
            y=0.0,
            z=0.890,
            length=1.22,
            radius=0.009,
            material=stainless,
            name=f"hinge_pin_{idx}",
        )
        base.visual(
            Box((0.130, 1.34, 0.014)),
            origin=Origin(xyz=(x, 0.0, 0.922), rpy=(0.0, -math.copysign(0.10, x), 0.0)),
            material=powder,
            name=f"rain_hood_{idx}",
        )
        base.visual(
            Box((0.018, 1.34, 0.030)),
            origin=Origin(xyz=(x + math.copysign(0.020, x), 0.0, 0.905)),
            material=dark_gasket,
            name=f"drip_edge_{idx}",
        )

    # Two continuous bent tube end frames and longitudinal ground feet keep the
    # stand mechanically continuous and anchored without detached legs.
    for idx, y in enumerate((-0.56, 0.56)):
        stand = wire_from_points(
            [
                (-0.36, y, 0.045),
                (-0.30, y, 0.34),
                (-0.215, y, 0.820),
                (0.215, y, 0.820),
                (0.30, y, 0.34),
                (0.36, y, 0.045),
            ],
            radius=0.015,
            radial_segments=18,
            cap_ends=True,
            corner_mode="fillet",
            corner_radius=0.055,
        )
        base.visual(
            mesh_from_geometry(stand, f"end_frame_{idx}"),
            material=powder,
            name=f"end_frame_{idx}",
        )

    for idx, x in enumerate((-0.36, 0.36)):
        _rod_y(
            base,
            x=x,
            y=0.0,
            z=0.032,
            length=1.20,
            radius=0.018,
            material=foot_rubber,
            name=f"foot_rail_{idx}",
        )
        for j, y in enumerate((-0.48, 0.48)):
            base.visual(
                Box((0.112, 0.105, 0.018)),
                origin=Origin(xyz=(x, y, 0.016)),
                material=foot_rubber,
                name=f"anchor_pad_{idx}_{j}",
            )
            base.visual(
                Cylinder(radius=0.015, length=0.006),
                origin=Origin(xyz=(x, y, 0.029)),
                material=stainless,
                name=f"anchor_bolt_{idx}_{j}",
            )

    # Positive and negative side wings are mirrored but use the same local
    # hinge convention: q=0 is deployed; positive q folds the free edge upward.
    wing_0 = model.part("wing_0")
    _add_wing_geometry(
        wing_0,
        sign=1.0,
        rail_material=powder,
        gasket_material=dark_gasket,
        hardware_material=stainless,
    )

    wing_1 = model.part("wing_1")
    _add_wing_geometry(
        wing_1,
        sign=-1.0,
        rail_material=powder,
        gasket_material=dark_gasket,
        hardware_material=stainless,
    )

    model.articulation(
        "base_to_wing_0",
        ArticulationType.REVOLUTE,
        parent=base,
        child=wing_0,
        origin=Origin(xyz=(0.255, 0.0, 0.890)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.2, lower=0.0, upper=1.22),
    )
    model.articulation(
        "base_to_wing_1",
        ArticulationType.REVOLUTE,
        parent=base,
        child=wing_1,
        origin=Origin(xyz=(-0.255, 0.0, 0.890)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.2, lower=0.0, upper=1.22),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    wing_0 = object_model.get_part("wing_0")
    wing_1 = object_model.get_part("wing_1")
    hinge_0 = object_model.get_articulation("base_to_wing_0")
    hinge_1 = object_model.get_articulation("base_to_wing_1")

    ctx.check(
        "two stopped wing hinges",
        hinge_0.motion_limits is not None
        and hinge_1.motion_limits is not None
        and hinge_0.motion_limits.lower == 0.0
        and hinge_1.motion_limits.lower == 0.0
        and 1.15 <= hinge_0.motion_limits.upper <= 1.30
        and 1.15 <= hinge_1.motion_limits.upper <= 1.30,
        details=f"limits: {hinge_0.motion_limits}, {hinge_1.motion_limits}",
    )

    # At the deployed stop the hinge boots sit just below the stainless pin
    # line and overlap it along the rack length, making the protected hinge
    # interface visible without using a collision allowance.
    for wing, pin_name in ((wing_0, "hinge_pin_1"), (wing_1, "hinge_pin_0")):
        ctx.expect_overlap(
            wing,
            base,
            axes="y",
            elem_a="hinge_boot",
            elem_b=pin_name,
            min_overlap=1.05,
            name=f"{wing.name} has continuous hinge-line coverage",
        )
        ctx.expect_gap(
            base,
            wing,
            axis="z",
            positive_elem=pin_name,
            negative_elem="hinge_boot",
            min_gap=0.001,
            max_gap=0.018,
            name=f"{wing.name} boot clears sealed pin",
        )

    rest_0 = ctx.part_element_world_aabb(wing_0, elem="outer_header")
    rest_1 = ctx.part_element_world_aabb(wing_1, elem="outer_header")
    with ctx.pose({hinge_0: 1.22, hinge_1: 1.22}):
        folded_0 = ctx.part_element_world_aabb(wing_0, elem="outer_header")
        folded_1 = ctx.part_element_world_aabb(wing_1, elem="outer_header")

    ctx.check(
        "wing_0 folds upward on its hinge line",
        rest_0 is not None
        and folded_0 is not None
        and folded_0[0][2] > rest_0[1][2] + 0.55,
        details=f"rest={rest_0}, folded={folded_0}",
    )
    ctx.check(
        "wing_1 folds upward on its hinge line",
        rest_1 is not None
        and folded_1 is not None
        and folded_1[0][2] > rest_1[1][2] + 0.55,
        details=f"rest={rest_1}, folded={folded_1}",
    )

    return ctx.report()


object_model = build_object_model()
