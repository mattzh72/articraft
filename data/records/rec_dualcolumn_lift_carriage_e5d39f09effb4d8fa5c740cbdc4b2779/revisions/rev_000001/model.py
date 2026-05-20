from __future__ import annotations

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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dual_column_lift_carriage")

    steel = model.material("brushed_steel", rgba=(0.68, 0.70, 0.70, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.12, 0.13, 0.14, 1.0))
    base_paint = model.material("matte_graphite", rgba=(0.05, 0.06, 0.07, 1.0))
    carriage_paint = model.material("machine_blue", rgba=(0.04, 0.20, 0.48, 1.0))
    stop_red = model.material("red_stop_collars", rgba=(0.78, 0.08, 0.04, 1.0))
    groove_black = model.material("black_recesses", rgba=(0.01, 0.01, 0.012, 1.0))

    def tube_mesh(name: str, outer_radius: float, inner_radius: float, length: float):
        """Open centered tube along local Z, with annular lips at each end."""
        return mesh_from_geometry(
            LatheGeometry.from_shell_profiles(
                [(outer_radius, -0.5 * length), (outer_radius, 0.5 * length)],
                [(inner_radius, -0.5 * length), (inner_radius, 0.5 * length)],
                segments=48,
                start_cap="flat",
                end_cap="flat",
            ),
            name,
        )

    frame = model.part("frame")
    frame.visual(
        Box((0.58, 0.32, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
        material=base_paint,
        name="base_slab",
    )
    frame.visual(
        Box((0.52, 0.12, 0.07)),
        origin=Origin(xyz=(0.0, 0.0, 0.595)),
        material=base_paint,
        name="top_bridge",
    )

    post_xs = (-0.18, 0.18)
    for index, x in enumerate(post_xs):
        frame.visual(
            Cylinder(radius=0.025, length=0.55),
            origin=Origin(xyz=(x, 0.0, 0.305)),
            material=steel,
            name=f"post_{index}",
        )
        frame.visual(
            Cylinder(radius=0.055, length=0.018),
            origin=Origin(xyz=(x, 0.0, 0.069)),
            material=dark_steel,
            name=f"base_socket_{index}",
        )
        frame.visual(
            tube_mesh(f"bottom_stop_mesh_{index}", 0.041, 0.023, 0.043),
            origin=Origin(xyz=(x, 0.0, 0.0985)),
            material=stop_red,
            name=f"bottom_stop_{index}",
        )
        frame.visual(
            tube_mesh(f"top_stop_mesh_{index}", 0.041, 0.023, 0.043),
            origin=Origin(xyz=(x, 0.0, 0.4815)),
            material=stop_red,
            name=f"top_stop_{index}",
        )
        frame.visual(
            Box((0.055, 0.024, 0.018)),
            origin=Origin(xyz=(x, 0.042, 0.0985)),
            material=stop_red,
            name=f"bottom_stop_tab_{index}",
        )
        frame.visual(
            Box((0.055, 0.024, 0.018)),
            origin=Origin(xyz=(x, 0.042, 0.4815)),
            material=stop_red,
            name=f"top_stop_tab_{index}",
        )

    carriage = model.part("carriage")
    for index, x in enumerate(post_xs):
        carriage.visual(
            tube_mesh(f"bearing_sleeve_mesh_{index}", 0.044, 0.032, 0.14),
            origin=Origin(xyz=(x, 0.0, 0.0)),
            material=dark_steel,
            name=f"bearing_sleeve_{index}",
        )
        for z in (-0.045, 0.045):
            carriage.visual(
                Box((0.105, 0.075, 0.030)),
                origin=Origin(xyz=(x, -0.072, z)),
                material=carriage_paint,
                name=f"sleeve_web_{index}_{'lower' if z < 0 else 'upper'}",
            )

    carriage.visual(
        Box((0.46, 0.040, 0.070)),
        origin=Origin(xyz=(0.0, -0.086, 0.0)),
        material=carriage_paint,
        name="backbone",
    )
    carriage.visual(
        Box((0.50, 0.036, 0.18)),
        origin=Origin(xyz=(0.0, -0.126, 0.0)),
        material=carriage_paint,
        name="tooling_plate",
    )
    carriage.visual(
        Box((0.42, 0.010, 0.018)),
        origin=Origin(xyz=(0.0, -0.146, 0.052)),
        material=groove_black,
        name="upper_t_slot",
    )
    carriage.visual(
        Box((0.42, 0.010, 0.018)),
        origin=Origin(xyz=(0.0, -0.146, -0.052)),
        material=groove_black,
        name="lower_t_slot",
    )
    for index, x in enumerate((-0.185, 0.0, 0.185)):
        for z in (-0.030, 0.030):
            carriage.visual(
                Cylinder(radius=0.011, length=0.008),
                origin=Origin(xyz=(x, -0.146, z), rpy=(1.57079632679, 0.0, 0.0)),
                material=groove_black,
                name=f"mount_bolt_{index}_{'lower' if z < 0 else 'upper'}",
            )

    model.articulation(
        "frame_to_carriage",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.19)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=600.0, velocity=0.18, lower=0.0, upper=0.20),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    carriage = object_model.get_part("carriage")
    slide = object_model.get_articulation("frame_to_carriage")

    ctx.check(
        "carriage travel is 200 mm",
        slide.motion_limits is not None
        and slide.motion_limits.lower == 0.0
        and abs(slide.motion_limits.upper - 0.20) < 1e-9,
        details=f"limits={slide.motion_limits}",
    )

    for index in (0, 1):
        sleeve = f"bearing_sleeve_{index}"
        post = f"post_{index}"
        bottom_stop = f"bottom_stop_{index}"
        top_stop = f"top_stop_{index}"
        ctx.expect_within(
            frame,
            carriage,
            axes="xy",
            inner_elem=post,
            outer_elem=sleeve,
            margin=0.008,
            name=f"post {index} passes through sleeve clearance",
        )
        ctx.expect_overlap(
            carriage,
            frame,
            axes="z",
            elem_a=sleeve,
            elem_b=post,
            min_overlap=0.13,
            name=f"lower pose sleeve {index} wraps the post",
        )
        ctx.expect_gap(
            carriage,
            frame,
            axis="z",
            positive_elem=sleeve,
            negative_elem=bottom_stop,
            max_gap=0.004,
            max_penetration=0.00001,
            name=f"bottom hard stop {index} is just below the sleeve",
        )

    rest_pos = ctx.part_world_position(carriage)
    with ctx.pose({slide: 0.20}):
        extended_pos = ctx.part_world_position(carriage)
        for index in (0, 1):
            sleeve = f"bearing_sleeve_{index}"
            post = f"post_{index}"
            top_stop = f"top_stop_{index}"
            ctx.expect_within(
                frame,
                carriage,
                axes="xy",
                inner_elem=post,
                outer_elem=sleeve,
                margin=0.008,
                name=f"raised post {index} remains inside sleeve clearance",
            )
            ctx.expect_overlap(
                carriage,
                frame,
                axes="z",
                elem_a=sleeve,
                elem_b=post,
                min_overlap=0.13,
                name=f"raised sleeve {index} remains wrapped around the post",
            )
            ctx.expect_gap(
                frame,
                carriage,
                axis="z",
                positive_elem=top_stop,
                negative_elem=sleeve,
                max_gap=0.004,
                max_penetration=0.00001,
                name=f"top hard stop {index} is just above the sleeve",
            )

    ctx.check(
        "carriage moves upward on the columns",
        rest_pos is not None
        and extended_pos is not None
        and extended_pos[2] > rest_pos[2] + 0.195,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    return ctx.report()


object_model = build_object_model()
