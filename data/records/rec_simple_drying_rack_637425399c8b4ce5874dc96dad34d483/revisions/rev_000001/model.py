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
    tube_from_spline_points,
)


def _rod_origin(xyz: tuple[float, float, float], axis: str) -> Origin:
    """Return an origin for a primitive cylinder whose local Z is along *axis*."""
    if axis == "x":
        return Origin(xyz=xyz, rpy=(0.0, math.pi / 2.0, 0.0))
    if axis == "y":
        return Origin(xyz=xyz, rpy=(math.pi / 2.0, 0.0, 0.0))
    return Origin(xyz=xyz)


def _add_rod(part, name: str, *, axis: str, center, length: float, radius: float, material) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=_rod_origin(center, axis),
        material=material,
        name=name,
    )


def _closed_tube(points, *, radius: float, name: str):
    return mesh_from_geometry(
        tube_from_spline_points(
            points,
            radius=radius,
            samples_per_segment=10,
            closed_spline=True,
            radial_segments=20,
            cap_ends=True,
        ),
        name,
    )


def _open_tube(points, *, radius: float, name: str):
    return mesh_from_geometry(
        tube_from_spline_points(
            points,
            radius=radius,
            samples_per_segment=12,
            closed_spline=False,
            radial_segments=20,
            cap_ends=True,
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_simple_drying_rack")

    painted_metal = model.material("warm_white_powder_coat", rgba=(0.86, 0.84, 0.78, 1.0))
    satin_rail = model.material("satin_anodized_rail", rgba=(0.72, 0.73, 0.70, 1.0))
    dark_polymer = model.material("graphite_polymer", rgba=(0.10, 0.11, 0.12, 1.0))
    hinge_metal = model.material("brushed_hinge_metal", rgba=(0.45, 0.46, 0.44, 1.0))
    elastomer = model.material("soft_black_elastomer", rgba=(0.02, 0.02, 0.018, 1.0))

    length = 1.18
    top_width = 0.42
    top_z = 0.82
    hinge_y = 0.255
    hinge_z = 0.84
    tube_r = 0.012
    rail_r = 0.0055

    base = model.part("base")

    # A continuous rounded rectangular top frame anchors the rack and gives the
    # central drying field a deliberate premium seam line.
    base.visual(
        _closed_tube(
            [
                (-length / 2.0, -top_width / 2.0, top_z),
                (length / 2.0, -top_width / 2.0, top_z),
                (length / 2.0, top_width / 2.0, top_z),
                (-length / 2.0, top_width / 2.0, top_z),
            ],
            radius=tube_r,
            name="base_top_frame_mesh",
        ),
        material=painted_metal,
        name="top_frame",
    )

    for i, y in enumerate((-0.145, -0.073, 0.0, 0.073, 0.145)):
        _add_rod(
            base,
            f"center_rail_{i}",
            axis="x",
            center=(0.0, y, top_z + 0.016),
            length=length - 0.005,
            radius=rail_r,
            material=satin_rail,
        )

    # Two end hoops and low floor rails make the base one physically connected
    # welded stand instead of a tabletop slab. The slight splay is typical of
    # stable domestic drying racks.
    for i, x in enumerate((-0.50, 0.50)):
        base.visual(
            _open_tube(
                [
                    (x, -0.315, 0.055),
                    (x, -0.245, 0.47),
                    (x, -0.205, top_z),
                    (x, 0.205, top_z),
                    (x, 0.245, 0.47),
                    (x, 0.315, 0.055),
                ],
                radius=0.011,
                name=f"leg_hoop_{i}_mesh",
            ),
            material=painted_metal,
            name=f"leg_hoop_{i}",
        )
        for j, y in enumerate((-0.315, 0.315)):
            base.visual(
                Box((0.115, 0.038, 0.060)),
                origin=Origin(xyz=(x, y, 0.030)),
                material=elastomer,
                name=f"foot_{i}_{j}",
            )

    for j, y in enumerate((-0.315, 0.315)):
        _add_rod(
            base,
            f"floor_rail_{j}",
            axis="x",
            center=(0.0, y, 0.055),
            length=1.00,
            radius=0.0085,
            material=painted_metal,
        )

    for i, x in enumerate((-0.585, 0.585)):
        base.visual(
            Box((0.038, 0.340, 0.024)),
            origin=Origin(xyz=(x, 0.0, top_z + 0.012)),
            material=dark_polymer,
            name=f"rail_manifold_{i}",
        )

    # Alternating hinge barrels are split along the rack length so the wing
    # barrels can occupy the gaps without interpenetration.
    base_knuckles = [(-0.45, 0.17), (0.0, 0.24), (0.45, 0.17)]
    for side, ysign in enumerate((1.0, -1.0)):
        y = ysign * hinge_y
        base.visual(
            Box((1.08, 0.038, 0.018)),
            origin=Origin(xyz=(0.0, y - ysign * 0.033, top_z + 0.008)),
            material=dark_polymer,
            name=f"hinge_spine_{side}",
        )
        for k, (x, seg_len) in enumerate(base_knuckles):
            _add_rod(
                base,
                f"hinge_barrel_{side}_{k}",
                axis="x",
                center=(x, y, hinge_z),
                length=seg_len,
                radius=0.0105,
                material=hinge_metal,
            )
            # Short bridge leaf from the painted side rail to each barrel.
            base.visual(
                Box((seg_len * 0.82, 0.050, 0.008)),
                origin=Origin(xyz=(x, y - ysign * 0.027, hinge_z - 0.003)),
                material=dark_polymer,
                name=f"hinge_bridge_{side}_{k}",
            )
        for s, x in enumerate((-0.28, 0.28)):
            base.visual(
                Box((0.080, 0.034, 0.012)),
                origin=Origin(xyz=(x, y - ysign * 0.055, top_z + 0.015)),
                material=elastomer,
                name=f"wing_stop_{side}_{s}",
            )

    def add_wing(name: str, sign: float, axis):
        wing = model.part(name)
        outer_y = sign * 0.53
        inner_y = sign * 0.065
        mid_y = sign * 0.29
        z = 0.020
        frame_len = length - 0.08

        wing.visual(
            _closed_tube(
                [
                    (-frame_len / 2.0, inner_y, z),
                    (frame_len / 2.0, inner_y, z),
                    (frame_len / 2.0, outer_y, z),
                    (-frame_len / 2.0, outer_y, z),
                ],
                radius=0.0105,
                name=f"{name}_frame_mesh",
            ),
            material=painted_metal,
            name="wing_frame",
        )

        for i, y_abs in enumerate((0.145, 0.235, 0.325, 0.415, 0.495)):
            _add_rod(
                wing,
                f"hanging_rail_{i}",
                axis="x",
                center=(0.0, sign * y_abs, z + 0.016),
                length=frame_len - 0.005,
                radius=rail_r,
                material=satin_rail,
            )

        for i, x in enumerate((-frame_len / 2.0, frame_len / 2.0)):
            wing.visual(
                Box((0.026, abs(outer_y - inner_y), 0.026)),
                origin=Origin(xyz=(x, (outer_y + inner_y) / 2.0, z + 0.010)),
                material=dark_polymer,
                name=f"rail_manifold_{i}",
            )

        # Polymer spacers at the far corners protect walls and give each wing a
        # crisp capped end without pretending the rods are floating.
        for i, x in enumerate((-frame_len / 2.0, frame_len / 2.0)):
            wing.visual(
                Box((0.052, 0.030, 0.030)),
                origin=Origin(xyz=(x, outer_y, z + 0.002)),
                material=dark_polymer,
                name=f"corner_cap_{i}",
            )

        # Wing-side hinge links: the link plates tie the alternating barrels to
        # the inner tube, while keeping a small visual seam from the fixed base
        # hinge bridges.
        wing_knuckles = [(-0.265, 0.145), (0.265, 0.145)]
        for k, (x, seg_len) in enumerate(wing_knuckles):
            _add_rod(
                wing,
                f"hinge_barrel_{k}",
                axis="x",
                center=(x, 0.0, 0.0),
                length=seg_len,
                radius=0.0105,
                material=hinge_metal,
            )
            wing.visual(
                Box((seg_len * 0.78, 0.086, 0.024)),
                origin=Origin(xyz=(x, sign * 0.034, 0.012)),
                material=dark_polymer,
                name=f"hinge_link_{k}",
            )

        # A slim stop tongue rests just above the base elastomer bumper at the
        # open position, making the 0 rad limit visually understandable.
        for k, x in enumerate((-0.28, 0.28)):
            wing.visual(
                Box((0.070, 0.044, 0.018)),
                origin=Origin(xyz=(x, sign * 0.060, 0.014)),
                material=dark_polymer,
                name=f"stop_tongue_{k}",
            )

        model.articulation(
            f"base_to_{name}",
            ArticulationType.REVOLUTE,
            parent=base,
            child=wing,
            origin=Origin(xyz=(0.0, sign * hinge_y, hinge_z)),
            axis=axis,
            motion_limits=MotionLimits(effort=7.0, velocity=1.2, lower=0.0, upper=1.75),
        )
        return wing

    add_wing("wing_0", 1.0, (1.0, 0.0, 0.0))
    add_wing("wing_1", -1.0, (-1.0, 0.0, 0.0))

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    wing_0 = object_model.get_part("wing_0")
    wing_1 = object_model.get_part("wing_1")
    hinge_0 = object_model.get_articulation("base_to_wing_0")
    hinge_1 = object_model.get_articulation("base_to_wing_1")

    ctx.expect_overlap(
        wing_0,
        base,
        axes="x",
        min_overlap=0.90,
        name="wing_0 spans the base hinge line",
    )
    ctx.expect_overlap(
        wing_1,
        base,
        axes="x",
        min_overlap=0.90,
        name="wing_1 spans the base hinge line",
    )

    rest_0 = ctx.part_element_world_aabb(wing_0, elem="hanging_rail_4")
    rest_1 = ctx.part_element_world_aabb(wing_1, elem="hanging_rail_4")
    with ctx.pose({hinge_0: 1.75, hinge_1: 1.75}):
        folded_0 = ctx.part_element_world_aabb(wing_0, elem="hanging_rail_4")
        folded_1 = ctx.part_element_world_aabb(wing_1, elem="hanging_rail_4")

    ctx.check(
        "wing_0 folds upward from the open stop",
        rest_0 is not None and folded_0 is not None and folded_0[0][2] > rest_0[1][2] + 0.30,
        details=f"rest={rest_0}, folded={folded_0}",
    )
    ctx.check(
        "wing_1 folds upward from the open stop",
        rest_1 is not None and folded_1 is not None and folded_1[0][2] > rest_1[1][2] + 0.30,
        details=f"rest={rest_1}, folded={folded_1}",
    )

    return ctx.report()


object_model = build_object_model()
