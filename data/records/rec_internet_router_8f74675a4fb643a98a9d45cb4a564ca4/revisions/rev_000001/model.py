from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    CylinderGeometry,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    MeshGeometry,
    MotionLimits,
    Origin,
    SlotPatternPanelGeometry,
    Sphere,
    TorusGeometry,
    mesh_from_geometry,
    TestContext,
    TestReport,
)


BODY_WIDTH = 0.56
BODY_DEPTH = 0.34
BODY_TOP_Z = 0.058
REAR_Y = BODY_DEPTH / 2 + 0.018
HINGE_Z = 0.087
SIDE_HINGE_X = BODY_WIDTH / 2 + 0.039
SIDE_HINGE_Y = 0.028
KNOB_X = 0.205
KNOB_Y = -0.112
KNOB_POD_TOP_Z = 0.075


def _octagon_loop(width: float, depth: float, corner: float, z: float) -> list[tuple[float, float, float]]:
    hx = width / 2
    hy = depth / 2
    c = min(corner, hx * 0.8, hy * 0.8)
    return [
        (-hx + c, -hy, z),
        (hx - c, -hy, z),
        (hx, -hy + c, z),
        (hx, hy - c, z),
        (hx - c, hy, z),
        (-hx + c, hy, z),
        (-hx, hy - c, z),
        (-hx, -hy + c, z),
    ]


def _faceted_housing_geometry() -> MeshGeometry:
    """Broad, low octagonal router shell with a sloped upper shoulder."""
    geom = MeshGeometry()
    loops = [
        _octagon_loop(BODY_WIDTH, BODY_DEPTH, 0.055, 0.0),
        _octagon_loop(BODY_WIDTH, BODY_DEPTH, 0.055, 0.040),
        _octagon_loop(0.485, 0.260, 0.045, BODY_TOP_Z),
    ]
    loop_ids: list[list[int]] = []
    for loop in loops:
        loop_ids.append([geom.add_vertex(x, y, z) for x, y, z in loop])

    count = len(loop_ids[0])
    for lower, upper in zip(loop_ids, loop_ids[1:]):
        for i in range(count):
            j = (i + 1) % count
            geom.add_face(lower[i], lower[j], upper[j])
            geom.add_face(lower[i], upper[j], upper[i])

    # Bottom and top caps.
    bottom_center = geom.add_vertex(0.0, 0.0, 0.0)
    for i in range(count):
        j = (i + 1) % count
        geom.add_face(bottom_center, loop_ids[0][j], loop_ids[0][i])
    top_center = geom.add_vertex(0.0, 0.0, BODY_TOP_Z)
    for i in range(count):
        j = (i + 1) % count
        geom.add_face(top_center, loop_ids[-1][i], loop_ids[-1][j])
    return geom


def _blade_geometry(width_axis: str) -> MeshGeometry:
    """Tapered flat antenna blade, rooted just above the hinge barrel."""
    geom = MeshGeometry()
    z0 = 0.010
    z1 = 0.330
    bottom_width = 0.040
    top_width = 0.026
    bottom_thick = 0.016
    top_thick = 0.011
    if width_axis == "x":
        dims = [
            (-bottom_width / 2, -bottom_thick / 2, z0),
            (bottom_width / 2, -bottom_thick / 2, z0),
            (bottom_width / 2, bottom_thick / 2, z0),
            (-bottom_width / 2, bottom_thick / 2, z0),
            (-top_width / 2, -top_thick / 2, z1),
            (top_width / 2, -top_thick / 2, z1),
            (top_width / 2, top_thick / 2, z1),
            (-top_width / 2, top_thick / 2, z1),
        ]
    else:
        dims = [
            (-bottom_thick / 2, -bottom_width / 2, z0),
            (bottom_thick / 2, -bottom_width / 2, z0),
            (bottom_thick / 2, bottom_width / 2, z0),
            (-bottom_thick / 2, bottom_width / 2, z0),
            (-top_thick / 2, -top_width / 2, z1),
            (top_thick / 2, -top_width / 2, z1),
            (top_thick / 2, top_width / 2, z1),
            (-top_thick / 2, top_width / 2, z1),
        ]
    ids = [geom.add_vertex(*p) for p in dims]
    faces = [
        (0, 1, 2), (0, 2, 3),  # bottom
        (4, 6, 5), (4, 7, 6),  # top
        (0, 4, 5), (0, 5, 1),
        (1, 5, 6), (1, 6, 2),
        (2, 6, 7), (2, 7, 3),
        (3, 7, 4), (3, 4, 0),
    ]
    for a, b, c in faces:
        geom.add_face(ids[a], ids[b], ids[c])
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="faceted_gaming_router")

    matte_black = model.material("matte_black", rgba=(0.015, 0.017, 0.020, 1.0))
    charcoal = model.material("charcoal_facets", rgba=(0.050, 0.055, 0.065, 1.0))
    rubber = model.material("soft_black_rubber", rgba=(0.006, 0.006, 0.007, 1.0))
    gunmetal = model.material("gunmetal_hinge", rgba=(0.16, 0.16, 0.17, 1.0))
    red = model.material("gaming_red", rgba=(0.85, 0.025, 0.010, 1.0))
    cool_blue = model.material("cool_blue_led", rgba=(0.0, 0.45, 1.0, 1.0))
    green = model.material("green_led", rgba=(0.05, 0.95, 0.20, 1.0))
    white = model.material("white_marking", rgba=(0.90, 0.92, 0.90, 1.0))

    housing = model.part("housing")
    housing.visual(
        mesh_from_geometry(_faceted_housing_geometry(), "faceted_shell"),
        material=matte_black,
        name="faceted_shell",
    )

    # Raised angular top treatment and perforated cooling fields.
    housing.visual(
        Box((0.210, 0.025, 0.004)),
        origin=Origin(xyz=(0.0, -0.010, BODY_TOP_Z + 0.0015), rpy=(0.0, 0.0, math.radians(18))),
        material=red,
        name="top_red_slash",
    )
    for x, name in [(-0.115, "top_vent_0"), (0.115, "top_vent_1")]:
        housing.visual(
            mesh_from_geometry(
                SlotPatternPanelGeometry(
                    (0.150, 0.072),
                    0.003,
                    slot_size=(0.040, 0.006),
                    pitch=(0.050, 0.015),
                    frame=0.010,
                    corner_radius=0.006,
                    slot_angle_deg=12.0 if x < 0 else -12.0,
                ),
                name,
            ),
            origin=Origin(xyz=(x, 0.032, BODY_TOP_Z + 0.001)),
            material=charcoal,
            name=name,
        )

    # Front status light strip.
    housing.visual(
        Box((0.170, 0.008, 0.009)),
        origin=Origin(xyz=(-0.045, -BODY_DEPTH / 2 - 0.001, 0.034)),
        material=charcoal,
        name="front_light_bar",
    )
    for i, (x, mat) in enumerate([(-0.105, cool_blue), (-0.070, green), (-0.035, green), (0.000, cool_blue)]):
        housing.visual(
            Box((0.016, 0.004, 0.005)),
            origin=Origin(xyz=(x, -BODY_DEPTH / 2 - 0.006, 0.036)),
            material=mat,
            name=f"status_led_{i}",
        )

    # Rotary corner pod with socket collar and mode ticks.
    housing.visual(
        mesh_from_geometry(
            # Twelve sides keep the pod visibly faceted rather than appliance-round.
            CylinderGeometry(0.052, 0.018, radial_segments=12),
            "corner_pod",
        ),
        origin=Origin(xyz=(KNOB_X, KNOB_Y, 0.066)),
        material=charcoal,
        name="corner_pod",
    )
    housing.visual(
        mesh_from_geometry(TorusGeometry(0.033, 0.003, radial_segments=18, tubular_segments=8), "knob_socket_ring"),
        origin=Origin(xyz=(KNOB_X, KNOB_Y, KNOB_POD_TOP_Z + 0.001)),
        material=gunmetal,
        name="knob_socket_ring",
    )
    for i, angle in enumerate((-45.0, 0.0, 45.0)):
        rad = math.radians(angle)
        housing.visual(
            Box((0.004, 0.018, 0.002)),
            origin=Origin(
                xyz=(KNOB_X + 0.043 * math.sin(rad), KNOB_Y + 0.043 * math.cos(rad), KNOB_POD_TOP_Z + 0.002),
                rpy=(0.0, 0.0, -rad),
            ),
            material=white,
            name=f"mode_tick_{i}",
        )

    # Rear yoke bases: four hinges along the rear edge.
    rear_xs = [-0.210, -0.070, 0.070, 0.210]
    for i, x in enumerate(rear_xs):
        housing.visual(
            Box((0.070, 0.052, 0.027)),
            origin=Origin(xyz=(x, REAR_Y - 0.004, 0.0575)),
            material=charcoal,
            name=f"rear_pod_{i}",
        )
        for side, sx in [("a", -1.0), ("b", 1.0)]:
            housing.visual(
                Box((0.008, 0.030, 0.036)),
                origin=Origin(xyz=(x + sx * 0.023, REAR_Y, HINGE_Z)),
                material=gunmetal,
                name=f"rear_yoke_{i}_{side}",
            )

    # Side yoke bases: one folding antenna on each side.
    for idx, sx in enumerate([-1.0, 1.0]):
        side_name = f"side_pod_{idx}"
        housing.visual(
            Box((0.066, 0.076, 0.027)),
            origin=Origin(xyz=(sx * (BODY_WIDTH / 2 + 0.011), SIDE_HINGE_Y, 0.0575)),
            material=charcoal,
            name=side_name,
        )
        for side, sy in [("a", -1.0), ("b", 1.0)]:
            housing.visual(
                Box((0.030, 0.008, 0.036)),
                origin=Origin(xyz=(sx * SIDE_HINGE_X, SIDE_HINGE_Y + sy * 0.023, HINGE_Z)),
                material=gunmetal,
                name=f"side_yoke_{idx}_{side}",
            )

    def add_antenna(
        name: str,
        *,
        hinge_xyz: tuple[float, float, float],
        hinge_axis: tuple[float, float, float],
        barrel_axis: str,
        fold_limit: float,
    ) -> None:
        antenna = model.part(name)
        if barrel_axis == "x":
            barrel_origin = Origin(rpy=(0.0, math.pi / 2.0, 0.0))
            blade_axis = "x"
            accent_size = (0.006, 0.003, 0.225)
            accent_origin = Origin(xyz=(0.0, -0.009, 0.185))
        else:
            barrel_origin = Origin(rpy=(math.pi / 2.0, 0.0, 0.0))
            blade_axis = "y"
            accent_size = (0.003, 0.006, 0.225)
            accent_origin = Origin(xyz=(-0.009 if hinge_axis[1] < 0.0 else 0.009, 0.0, 0.185))
        antenna.visual(
            Cylinder(radius=0.014, length=0.038),
            origin=barrel_origin,
            material=gunmetal,
            name="hinge_barrel",
        )
        antenna.visual(
            mesh_from_geometry(_blade_geometry(blade_axis), f"{name}_blade"),
            material=rubber,
            name="tapered_blade",
        )
        antenna.visual(
            Box(accent_size),
            origin=accent_origin,
            material=red,
            name="red_inlay",
        )
        antenna.visual(
            Sphere(0.012),
            origin=Origin(xyz=(0.0, 0.0, 0.330)),
            material=rubber,
            name="rounded_tip",
        )
        model.articulation(
            f"housing_to_{name}",
            ArticulationType.REVOLUTE,
            parent=housing,
            child=antenna,
            origin=Origin(xyz=hinge_xyz),
            axis=hinge_axis,
            motion_limits=MotionLimits(effort=2.0, velocity=2.0, lower=-0.15, upper=fold_limit),
        )

    for i, x in enumerate(rear_xs):
        # Positive motion folds rear antennas backward over the rear edge.
        add_antenna(
            f"rear_antenna_{i}",
            hinge_xyz=(x, REAR_Y, HINGE_Z),
            hinge_axis=(-1.0, 0.0, 0.0),
            barrel_axis="x",
            fold_limit=1.65,
        )

    # side_antenna_0 is on -X and side_antenna_1 is on +X; positive motion folds outward.
    add_antenna(
        "side_antenna_0",
        hinge_xyz=(-SIDE_HINGE_X, SIDE_HINGE_Y, HINGE_Z),
        hinge_axis=(0.0, -1.0, 0.0),
        barrel_axis="y",
        fold_limit=1.55,
    )
    add_antenna(
        "side_antenna_1",
        hinge_xyz=(SIDE_HINGE_X, SIDE_HINGE_Y, HINGE_Z),
        hinge_axis=(0.0, 1.0, 0.0),
        barrel_axis="y",
        fold_limit=1.55,
    )

    mode_knob = model.part("mode_knob")
    mode_knob.visual(
        Cylinder(radius=0.010, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=gunmetal,
        name="short_shaft",
    )
    mode_knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.048,
                0.026,
                body_style="faceted",
                base_diameter=0.052,
                top_diameter=0.039,
                edge_radius=0.0008,
                grip=KnobGrip(style="ribbed", count=12, depth=0.0012, width=0.0020),
                indicator=KnobIndicator(style="wedge", mode="raised", angle_deg=0.0),
                center=False,
            ),
            "mode_knob_cap",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=rubber,
        name="knob_cap",
    )
    mode_knob.visual(
        Box((0.005, 0.020, 0.002)),
        origin=Origin(xyz=(0.0, 0.012, 0.038)),
        material=red,
        name="pointer_mark",
    )
    model.articulation(
        "housing_to_mode_knob",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=mode_knob,
        origin=Origin(xyz=(KNOB_X, KNOB_Y, KNOB_POD_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.4, velocity=5.0, lower=-math.pi, upper=math.pi),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    antenna_names = [f"rear_antenna_{i}" for i in range(4)] + ["side_antenna_0", "side_antenna_1"]
    ctx.check(
        "six folding antennas are modeled",
        all(object_model.get_part(name) is not None for name in antenna_names),
        details=f"antenna parts={antenna_names}",
    )

    for i in range(4):
        antenna = object_model.get_part(f"rear_antenna_{i}")
        hinge = object_model.get_articulation(f"housing_to_rear_antenna_{i}")
        housing = object_model.get_part("housing")
        ctx.expect_contact(
            antenna,
            housing,
            elem_a="hinge_barrel",
            elem_b=f"rear_yoke_{i}_a",
            contact_tol=0.001,
            name=f"rear antenna {i} is captured by one yoke cheek",
        )
        rest_aabb = ctx.part_element_world_aabb(antenna, elem="tapered_blade")
        with ctx.pose({hinge: 1.0}):
            folded_aabb = ctx.part_element_world_aabb(antenna, elem="tapered_blade")
        ctx.check(
            f"rear antenna {i} folds toward the rear",
            rest_aabb is not None
            and folded_aabb is not None
            and folded_aabb[1][1] > rest_aabb[1][1] + 0.05,
            details=f"rest={rest_aabb}, folded={folded_aabb}",
        )

    for idx, outward_axis in [(0, -1.0), (1, 1.0)]:
        antenna = object_model.get_part(f"side_antenna_{idx}")
        hinge = object_model.get_articulation(f"housing_to_side_antenna_{idx}")
        housing = object_model.get_part("housing")
        ctx.expect_contact(
            antenna,
            housing,
            elem_a="hinge_barrel",
            elem_b=f"side_yoke_{idx}_a",
            contact_tol=0.001,
            name=f"side antenna {idx} is captured by one yoke cheek",
        )
        rest_aabb = ctx.part_element_world_aabb(antenna, elem="tapered_blade")
        with ctx.pose({hinge: 0.9}):
            folded_aabb = ctx.part_element_world_aabb(antenna, elem="tapered_blade")
        if outward_axis < 0:
            ok = rest_aabb is not None and folded_aabb is not None and folded_aabb[0][0] < rest_aabb[0][0] - 0.04
        else:
            ok = rest_aabb is not None and folded_aabb is not None and folded_aabb[1][0] > rest_aabb[1][0] + 0.04
        ctx.check(
            f"side antenna {idx} folds outward",
            ok,
            details=f"rest={rest_aabb}, folded={folded_aabb}",
        )

    housing = object_model.get_part("housing")
    mode_knob = object_model.get_part("mode_knob")
    knob_joint = object_model.get_articulation("housing_to_mode_knob")
    ctx.expect_contact(
        mode_knob,
        housing,
        elem_a="short_shaft",
        elem_b="corner_pod",
        contact_tol=0.001,
        name="mode knob shaft sits on the corner pod",
    )
    rest_pointer = ctx.part_element_world_aabb(mode_knob, elem="pointer_mark")
    with ctx.pose({knob_joint: math.pi / 2.0}):
        turned_pointer = ctx.part_element_world_aabb(mode_knob, elem="pointer_mark")
    ctx.check(
        "mode knob pointer rotates around its own shaft",
        rest_pointer is not None
        and turned_pointer is not None
        and abs((turned_pointer[0][0] + turned_pointer[1][0]) - (rest_pointer[0][0] + rest_pointer[1][0])) > 0.015,
        details=f"rest={rest_pointer}, turned={turned_pointer}",
    )

    return ctx.report()


object_model = build_object_model()
