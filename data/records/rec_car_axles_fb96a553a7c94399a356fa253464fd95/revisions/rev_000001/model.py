from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rear_beam_axle")

    painted_steel = Material("satin_black_painted_steel", rgba=(0.03, 0.035, 0.038, 1.0))
    worn_edges = Material("worn_dark_steel", rgba=(0.18, 0.18, 0.17, 1.0))
    machined = Material("machined_bearing_steel", rgba=(0.55, 0.55, 0.52, 1.0))
    shadow = Material("black_recesses", rgba=(0.005, 0.005, 0.004, 1.0))

    x_cyl = (0.0, math.pi / 2.0, 0.0)

    housing = model.part("housing")
    housing.visual(
        Box((1.54, 0.13, 0.14)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=painted_steel,
        name="main_beam",
    )
    for sign, suffix in ((-1.0, "0"), (1.0, "1")):
        housing.visual(
            Box((0.040, 0.158, 0.158)),
            origin=Origin(xyz=(sign * 0.790, 0.0, 0.0)),
            material=worn_edges,
            name=f"end_plate_{suffix}",
        )
        housing.visual(
            Cylinder(radius=0.072, length=0.012),
            origin=Origin(xyz=(sign * 0.804, 0.0, 0.0), rpy=x_cyl),
            material=machined,
            name=f"spindle_face_{suffix}",
        )

    for sign, suffix in ((-1.0, "0"), (1.0, "1")):
        bearing = model.part(f"bearing_{suffix}")
        bearing.visual(
            Cylinder(radius=0.105, length=0.100),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=x_cyl),
            material=machined,
            name="outer_race",
        )
        bearing.visual(
            Cylinder(radius=0.074, length=0.008),
            origin=Origin(xyz=(sign * 0.050, 0.0, 0.0), rpy=x_cyl),
            material=shadow,
            name="bearing_seal",
        )
        bearing.visual(
            Cylinder(radius=0.111, length=0.014),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=x_cyl),
            material=worn_edges,
            name="retaining_band",
        )
        model.articulation(
            f"housing_to_bearing_{suffix}",
            ArticulationType.FIXED,
            parent=housing,
            child=bearing,
            origin=Origin(xyz=(sign * 0.860, 0.0, 0.0)),
        )

        hub = model.part(f"hub_{suffix}")
        hub.visual(
            Cylinder(radius=0.130, length=0.055),
            origin=Origin(xyz=(sign * 0.0275, 0.0, 0.0), rpy=x_cyl),
            material=worn_edges,
            name="wheel_flange",
        )
        hub.visual(
            Cylinder(radius=0.067, length=0.072),
            origin=Origin(xyz=(sign * 0.091, 0.0, 0.0), rpy=x_cyl),
            material=machined,
            name="center_pilot",
        )
        hub.visual(
            Cylinder(radius=0.038, length=0.035),
            origin=Origin(xyz=(sign * 0.1445, 0.0, 0.0), rpy=x_cyl),
            material=worn_edges,
            name="dust_cap",
        )
        for lug_index in range(5):
            angle = 2.0 * math.pi * lug_index / 5.0 + math.pi / 2.0
            hub.visual(
                Cylinder(radius=0.011, length=0.036),
                origin=Origin(
                    xyz=(
                        sign * 0.073,
                        0.094 * math.cos(angle),
                        0.094 * math.sin(angle),
                    ),
                    rpy=x_cyl,
                ),
                material=machined,
                name=f"lug_stud_{lug_index}",
            )
        model.articulation(
            f"bearing_to_hub_{suffix}",
            ArticulationType.CONTINUOUS,
            parent=bearing,
            child=hub,
            origin=Origin(xyz=(sign * 0.050, 0.0, 0.0)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=1200.0, velocity=80.0),
        )

    for sign, suffix in ((-1.0, "0"), (1.0, "1")):
        saddle = model.part(f"saddle_{suffix}")
        saddle.visual(
            Box((0.250, 0.220, 0.026)),
            origin=Origin(xyz=(0.0, 0.0, 0.013)),
            material=worn_edges,
            name="top_plate",
        )
        for y_sign, side_name in ((-1.0, "front"), (1.0, "rear")):
            saddle.visual(
                Box((0.225, 0.012, 0.062)),
                origin=Origin(xyz=(0.0, y_sign * 0.071, -0.030)),
                material=painted_steel,
                name=f"{side_name}_skirt",
            )
            saddle.visual(
                Box((0.230, 0.010, 0.010)),
                origin=Origin(xyz=(0.0, y_sign * 0.075, 0.004)),
                material=painted_steel,
                name=f"{side_name}_weld",
            )
        saddle.visual(
            Cylinder(radius=0.022, length=0.005),
            origin=Origin(xyz=(0.0, 0.0, 0.028)),
            material=shadow,
            name="center_bolt_recess",
        )
        for hole_index, (local_x, local_y) in enumerate(
            ((-0.078, -0.064), (-0.078, 0.064), (0.078, -0.064), (0.078, 0.064))
        ):
            saddle.visual(
                Cylinder(radius=0.014, length=0.004),
                origin=Origin(xyz=(local_x, local_y, 0.028)),
                material=shadow,
                name=f"u_bolt_hole_{hole_index}",
            )
        model.articulation(
            f"housing_to_saddle_{suffix}",
            ArticulationType.FIXED,
            parent=housing,
            child=saddle,
            origin=Origin(xyz=(sign * 0.420, 0.0, 0.070)),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")

    continuous_hubs = []
    for suffix in ("0", "1"):
        bearing = object_model.get_part(f"bearing_{suffix}")
        hub = object_model.get_part(f"hub_{suffix}")
        joint = object_model.get_articulation(f"bearing_to_hub_{suffix}")
        continuous_hubs.append(joint.articulation_type == ArticulationType.CONTINUOUS)
        ctx.check(
            f"hub {suffix} bearing axis runs along the axle beam",
            tuple(round(v, 6) for v in joint.axis) == (1.0, 0.0, 0.0),
            details=f"axis={joint.axis}",
        )

        if suffix == "0":
            ctx.expect_gap(
                bearing,
                hub,
                axis="x",
                positive_elem="outer_race",
                negative_elem="wheel_flange",
                max_gap=0.0015,
                max_penetration=0.0,
                name="negative hub flange seats against its bearing",
            )
        else:
            ctx.expect_gap(
                hub,
                bearing,
                axis="x",
                positive_elem="wheel_flange",
                negative_elem="outer_race",
                max_gap=0.0015,
                max_penetration=0.0,
                name="positive hub flange seats against its bearing",
            )

        rest_position = ctx.part_world_position(hub)
        with ctx.pose({joint: math.pi / 2.0}):
            turned_position = ctx.part_world_position(hub)
        ctx.check(
            f"hub {suffix} rotates without translating",
            rest_position is not None
            and turned_position is not None
            and all(abs(a - b) < 1e-6 for a, b in zip(rest_position, turned_position)),
            details=f"rest={rest_position}, turned={turned_position}",
        )

    ctx.check(
        "both wheel hubs use continuous bearings",
        all(continuous_hubs) and len(continuous_hubs) == 2,
        details=f"continuous_flags={continuous_hubs}",
    )

    for suffix in ("0", "1"):
        saddle = object_model.get_part(f"saddle_{suffix}")
        ctx.expect_gap(
            saddle,
            housing,
            axis="z",
            positive_elem="top_plate",
            negative_elem="main_beam",
            max_gap=0.001,
            max_penetration=0.0,
            name=f"saddle {suffix} sits on top of the beam",
        )
        ctx.expect_overlap(
            saddle,
            housing,
            axes="xy",
            elem_a="top_plate",
            elem_b="main_beam",
            min_overlap=0.120,
            name=f"saddle {suffix} spans across the beam top",
        )

    return ctx.report()


object_model = build_object_model()
