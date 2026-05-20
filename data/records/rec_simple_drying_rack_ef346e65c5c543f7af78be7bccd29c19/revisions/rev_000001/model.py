from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    ClevisBracketGeometry,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="retrofit_foldable_drying_rack")

    galvanized = model.material("dull_galvanized_steel", rgba=(0.62, 0.64, 0.60, 1.0))
    enamel = model.material("cream_chipped_enamel", rgba=(0.86, 0.82, 0.68, 1.0))
    dark_enamel = model.material("olive_enamel_adapters", rgba=(0.25, 0.33, 0.25, 1.0))
    brass = model.material("aged_brass_pins", rgba=(0.75, 0.58, 0.30, 1.0))
    rubber = model.material("black_rubber_feet", rgba=(0.02, 0.02, 0.018, 1.0))
    hatch_red = model.material("service_hatch_red", rgba=(0.55, 0.13, 0.08, 1.0))

    def cyl_x(part, name, length, radius, xyz, material, rpy=(0.0, math.pi / 2.0, 0.0)):
        part.visual(
            Cylinder(radius=radius, length=length),
            origin=Origin(xyz=xyz, rpy=rpy),
            material=material,
            name=name,
        )

    def cyl_y(part, name, length, radius, xyz, material):
        part.visual(
            Cylinder(radius=radius, length=length),
            origin=Origin(xyz=xyz, rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=material,
            name=name,
        )

    def cyl_z(part, name, length, radius, xyz, material):
        part.visual(
            Cylinder(radius=radius, length=length),
            origin=Origin(xyz=xyz),
            material=material,
            name=name,
        )

    def add_bolt_head(part, name, xyz, radius=0.010, height=0.006, axis="y"):
        rpy = (0.0, 0.0, 0.0)
        if axis == "x":
            rpy = (0.0, math.pi / 2.0, 0.0)
        elif axis == "y":
            rpy = (-math.pi / 2.0, 0.0, 0.0)
        part.visual(
            Cylinder(radius=radius, length=height),
            origin=Origin(xyz=xyz, rpy=rpy),
            material=brass,
            name=name,
        )

    def add_wing(part, y_sign: float):
        side = "pos" if y_sign > 0.0 else "neg"
        # A real drying rack wing is a welded rectangular tube with rods running
        # through side rails.  All rods slightly enter the frame tubes so the
        # authored part is one continuous supported assembly.
        cyl_x(part, "hinge_barrel", 1.08, 0.014, (0.0, 0.0, 0.0), brass)
        cyl_x(part, "outer_tube", 1.04, 0.011, (0.0, y_sign * 0.50, 0.0), galvanized)
        cyl_y(part, "end_tube_0", 0.47, 0.010, (-0.52, y_sign * 0.265, 0.0), galvanized)
        cyl_y(part, "end_tube_1", 0.47, 0.010, (0.52, y_sign * 0.265, 0.0), galvanized)
        for i, y in enumerate((0.10, 0.19, 0.28, 0.37, 0.45)):
            cyl_x(part, f"hanging_rail_{i}", 1.02, 0.006, (0.0, y_sign * y, 0.002), galvanized)
        part.visual(
            Box((0.110, 0.060, 0.016)),
            origin=Origin(xyz=(0.0, y_sign * 0.275, -0.052)),
            material=dark_enamel,
            name="link_socket",
        )
        for x_web in (-0.065, 0.065):
            part.visual(
                Box((0.030, 0.095, 0.062)),
                origin=Origin(xyz=(x_web, y_sign * 0.275, -0.017)),
                material=dark_enamel,
                name=f"link_socket_web_{'a' if x_web < 0 else 'b'}",
            )
        # Old retrofit wing corners use welded tabs rather than perfectly clean
        # modern molded joints.
        for x in (-0.52, 0.52):
            part.visual(
                Box((0.045, 0.070, 0.012)),
                origin=Origin(xyz=(x, y_sign * 0.035, -0.010)),
                material=dark_enamel,
                name=f"hinge_adapter_{side}_{'a' if x < 0 else 'b'}",
            )
            part.visual(
                Box((0.070, 0.040, 0.012)),
                origin=Origin(xyz=(x, y_sign * 0.235, -0.012)),
                material=dark_enamel,
                name=f"link_stop_{side}_{'a' if x < 0 else 'b'}",
            )
            add_bolt_head(
                part,
                f"wing_bolt_{side}_{'a' if x < 0 else 'b'}",
                (x, y_sign * 0.036, -0.018),
                radius=0.007,
                height=0.006,
                axis="z",
            )

    core = model.part("core")
    # Central rectangular deck and hanging rails.
    cyl_x(core, "side_rail_0", 1.16, 0.013, (0.0, -0.235, 0.900), galvanized)
    cyl_x(core, "side_rail_1", 1.16, 0.013, (0.0, 0.235, 0.900), galvanized)
    for i, x in enumerate((-0.54, 0.0, 0.54)):
        cyl_y(core, f"cross_tube_{i}", 0.50, 0.011, (x, 0.0, 0.900), galvanized)
    for i, y in enumerate((-0.16, -0.08, 0.0, 0.08, 0.16)):
        cyl_x(core, f"center_hanging_rail_{i}", 1.06, 0.006, (0.0, y, 0.906), galvanized)

    # Fold hinge pins mounted on bolted retrofit adapter rails.
    for side_index, y in enumerate((-0.305, 0.305)):
        side = "neg" if y < 0.0 else "pos"
        core.visual(
            Box((1.18, 0.030, 0.055)),
            origin=Origin(xyz=(0.0, y, 0.845)),
            material=dark_enamel,
            name=f"adapter_rail_{side}",
        )
        core.visual(
            Box((1.18, 0.030, 0.060)),
            origin=Origin(xyz=(0.0, 0.250 if y > 0.0 else -0.250, 0.870)),
            material=dark_enamel,
            name=f"adapter_drop_{side}",
        )
        core.visual(
            Box((1.18, 0.060, 0.018)),
            origin=Origin(xyz=(0.0, 0.278 if y > 0.0 else -0.278, 0.850)),
            material=dark_enamel,
            name=f"adapter_bridge_{side}",
        )
        cyl_x(core, f"hinge_pin_{side}", 1.12, 0.006, (0.0, y, 0.900), brass)
        for x_end in (-0.565, 0.565):
            core.visual(
                Box((0.012, 0.030, 0.060)),
                origin=Origin(xyz=(x_end, y, 0.872)),
                material=dark_enamel,
                name=f"pin_stanchion_{side}_{'a' if x_end < 0 else 'b'}",
            )
            core.visual(
                Box((0.018, 0.034, 0.024)),
                origin=Origin(xyz=(x_end, y, 0.902)),
                material=brass,
                name=f"pin_retainer_{side}_{'a' if x_end < 0 else 'b'}",
            )
        for x in (-0.48, -0.16, 0.16, 0.48):
            add_bolt_head(core, f"adapter_bolt_{side}_{x:+.2f}", (x, y + (0.013 if y > 0 else -0.013), 0.882))
        # Small cast stop blocks give the wings a hard open stop.
        for x in (-0.42, 0.42):
            core.visual(
                Box((0.070, 0.040, 0.035)),
                origin=Origin(xyz=(x, y * 0.97, 0.835)),
                material=dark_enamel,
                name=f"fold_stop_{side}_{'a' if x < 0 else 'b'}",
            )

    # Four straight legs and lower braces.  Their old-school utilitarian build
    # makes the rack self-supporting without any floating decorative pieces.
    for x in (-0.52, 0.52):
        for y in (-0.22, 0.22):
            tag = f"{'neg' if x < 0 else 'pos'}_{'neg' if y < 0 else 'pos'}"
            cyl_z(core, f"leg_{tag}", 0.64, 0.012, (x, y, 0.580), galvanized)
            cyl_z(core, f"rubber_foot_{tag}", 0.036, 0.022, (x, y, 0.2425), rubber)
    for y in (-0.22, 0.22):
        cyl_x(core, f"lower_long_brace_{'neg' if y < 0 else 'pos'}", 1.06, 0.008, (0.0, y, 0.355), galvanized)
    for x in (-0.52, 0.52):
        cyl_y(core, f"lower_cross_brace_{'neg' if x < 0 else 'pos'}", 0.44, 0.008, (x, 0.0, 0.390), galvanized)

    # Service hatches and labels are bolted to the adapter spine instead of
    # floating.  The small raised patch panels read as legacy repair access.
    for y in (-0.324, 0.324):
        side = "neg" if y < 0.0 else "pos"
        core.visual(
            Box((0.265, 0.010, 0.105)),
            origin=Origin(xyz=(-0.23, y, 0.805)),
            material=hatch_red,
            name=f"service_hatch_{side}",
        )
        core.visual(
            Box((0.285, 0.006, 0.012)),
            origin=Origin(xyz=(-0.23, y, 0.863)),
            material=brass,
            name=f"hatch_hinge_{side}",
        )
        for x in (-0.335, -0.125):
            for z in (0.775, 0.835):
                add_bolt_head(core, f"hatch_bolt_{side}_{x:+.2f}_{z:.2f}", (x, y + (0.006 if y > 0 else -0.006), z), radius=0.006, height=0.005)

    # Center posts carry the folding support links; clevis meshes give the
    # assembly pragmatic serviceable pivot brackets.
    clevis_mesh = mesh_from_geometry(
        ClevisBracketGeometry(
            (0.080, 0.050, 0.070),
            gap_width=0.048,
            bore_diameter=0.014,
            bore_center_z=0.045,
            base_thickness=0.012,
            corner_radius=0.004,
        ),
        "support_clevis",
    )
    for y in (-0.235, 0.235):
        side = "neg" if y < 0.0 else "pos"
        for x_post in (-0.045, 0.045):
            cyl_z(
                core,
                f"link_post_{side}_{'a' if x_post < 0 else 'b'}",
                0.185,
                0.008,
                (x_post, y, 0.765),
                galvanized,
            )
        core.visual(
            Box((0.070, 0.060, 0.022)),
            origin=Origin(xyz=(0.0, y, 0.660)),
            material=dark_enamel,
            name=f"clevis_mount_{side}",
        )
        core.visual(
            clevis_mesh,
            origin=Origin(xyz=(0.0, y, 0.680)),
            material=dark_enamel,
            name=f"link_clevis_{side}",
        )

    wing_0 = model.part("wing_0")
    add_wing(wing_0, 1.0)
    wing_1 = model.part("wing_1")
    add_wing(wing_1, -1.0)

    def add_hinge_link(part, y_sign: float):
        # Flat folding strap with a reinforced pivot boss and a rubberized stop
        # pad at its far end.  The strap is a separate articulated link, while
        # the stop pad bears against the wing stop tab in the open pose.
        roll = math.atan2(0.165, 0.305) if y_sign > 0.0 else math.pi - math.atan2(0.165, 0.305)
        part.visual(
            Box((0.028, 0.350, 0.014)),
            origin=Origin(xyz=(0.0, y_sign * 0.152, 0.083), rpy=(roll, 0.0, 0.0)),
            material=dark_enamel,
            name="flat_strap",
        )
        cyl_x(part, "pivot_boss", 0.060, 0.018, (0.0, 0.0, 0.0), brass)
        part.visual(
            Box((0.070, 0.035, 0.018)),
            origin=Origin(xyz=(0.0, y_sign * 0.305, 0.165), rpy=(roll, 0.0, 0.0)),
            material=rubber,
            name="stop_pad",
        )
        for i, s in enumerate((0.10, 0.23)):
            part.visual(
                Cylinder(radius=0.007, length=0.006),
                origin=Origin(xyz=(0.0, y_sign * s, 0.165 * (s / 0.305)), rpy=(math.pi / 2.0, 0.0, 0.0)),
                material=brass,
                name=f"strap_rivet_{i}",
            )

    hinge_link_0 = model.part("hinge_link_0")
    add_hinge_link(hinge_link_0, 1.0)
    hinge_link_1 = model.part("hinge_link_1")
    add_hinge_link(hinge_link_1, -1.0)

    model.articulation(
        "core_to_wing_0",
        ArticulationType.REVOLUTE,
        parent=core,
        child=wing_0,
        origin=Origin(xyz=(0.0, 0.305, 0.900)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.2, lower=0.0, upper=1.35),
    )
    model.articulation(
        "core_to_wing_1",
        ArticulationType.REVOLUTE,
        parent=core,
        child=wing_1,
        origin=Origin(xyz=(0.0, -0.305, 0.900)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.2, lower=0.0, upper=1.35),
    )
    model.articulation(
        "core_to_link_0",
        ArticulationType.REVOLUTE,
        parent=core,
        child=hinge_link_0,
        origin=Origin(xyz=(0.0, 0.235, 0.690)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=1.0, lower=-0.45, upper=0.65),
    )
    model.articulation(
        "core_to_link_1",
        ArticulationType.REVOLUTE,
        parent=core,
        child=hinge_link_1,
        origin=Origin(xyz=(0.0, -0.235, 0.690)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=1.0, lower=-0.45, upper=0.65),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    core = object_model.get_part("core")
    wing_0 = object_model.get_part("wing_0")
    wing_1 = object_model.get_part("wing_1")
    link_0 = object_model.get_part("hinge_link_0")
    link_1 = object_model.get_part("hinge_link_1")
    wing_joint_0 = object_model.get_articulation("core_to_wing_0")
    wing_joint_1 = object_model.get_articulation("core_to_wing_1")
    link_joint_0 = object_model.get_articulation("core_to_link_0")
    link_joint_1 = object_model.get_articulation("core_to_link_1")

    # The modeled hinge barrels are solid proxy cylinders around solid pin
    # proxies.  That local overlap is intentional: it represents a captured pin
    # running through a hollow hinge barrel.
    ctx.allow_overlap(
        core,
        wing_0,
        elem_a="hinge_pin_pos",
        elem_b="hinge_barrel",
        reason="The brass hinge pin is intentionally captured inside the wing barrel proxy.",
    )
    ctx.expect_within(
        core,
        wing_0,
        axes="yz",
        inner_elem="hinge_pin_pos",
        outer_elem="hinge_barrel",
        margin=0.001,
        name="positive wing pin is concentric with barrel",
    )
    ctx.expect_overlap(
        core,
        wing_0,
        axes="x",
        elem_a="hinge_pin_pos",
        elem_b="hinge_barrel",
        min_overlap=1.05,
        name="positive wing hinge has full length engagement",
    )
    ctx.allow_overlap(
        core,
        wing_1,
        elem_a="hinge_pin_neg",
        elem_b="hinge_barrel",
        reason="The brass hinge pin is intentionally captured inside the wing barrel proxy.",
    )
    ctx.expect_within(
        core,
        wing_1,
        axes="yz",
        inner_elem="hinge_pin_neg",
        outer_elem="hinge_barrel",
        margin=0.001,
        name="negative wing pin is concentric with barrel",
    )
    ctx.expect_overlap(
        core,
        wing_1,
        axes="x",
        elem_a="hinge_pin_neg",
        elem_b="hinge_barrel",
        min_overlap=1.05,
        name="negative wing hinge has full length engagement",
    )

    # Likewise the link pivot boss is shown as a captured pin/bushing passing
    # through the clevis bracket rather than as a visually hollowed collision
    # shell.
    ctx.allow_overlap(
        core,
        link_0,
        elem_a="link_clevis_pos",
        elem_b="pivot_boss",
        reason="The link pivot boss is intentionally captured in the clevis bore.",
    )
    ctx.expect_overlap(
        core,
        link_0,
        axes="x",
        elem_a="link_clevis_pos",
        elem_b="pivot_boss",
        min_overlap=0.040,
        name="positive support link is retained by clevis",
    )
    ctx.allow_overlap(
        core,
        link_1,
        elem_a="link_clevis_neg",
        elem_b="pivot_boss",
        reason="The link pivot boss is intentionally captured in the clevis bore.",
    )
    ctx.expect_overlap(
        core,
        link_1,
        axes="x",
        elem_a="link_clevis_neg",
        elem_b="pivot_boss",
        min_overlap=0.040,
        name="negative support link is retained by clevis",
    )

    ctx.allow_overlap(
        wing_0,
        link_0,
        elem_a="link_socket",
        elem_b="stop_pad",
        reason="The rubber support-link pad is seated in the wing socket at the open stop.",
    )
    ctx.expect_overlap(
        wing_0,
        link_0,
        axes="xy",
        elem_a="link_socket",
        elem_b="stop_pad",
        min_overlap=0.008,
        name="positive support link seats in wing socket",
    )
    ctx.allow_overlap(
        wing_1,
        link_1,
        elem_a="link_socket",
        elem_b="stop_pad",
        reason="The rubber support-link pad is seated in the wing socket at the open stop.",
    )
    ctx.expect_overlap(
        wing_1,
        link_1,
        axes="xy",
        elem_a="link_socket",
        elem_b="stop_pad",
        min_overlap=0.008,
        name="negative support link seats in wing socket",
    )

    ctx.check(
        "wing hinges have realistic stop limits",
        wing_joint_0.motion_limits.lower == 0.0
        and wing_joint_1.motion_limits.lower == 0.0
        and 1.2 <= wing_joint_0.motion_limits.upper <= 1.5
        and 1.2 <= wing_joint_1.motion_limits.upper <= 1.5,
        details=f"limits={wing_joint_0.motion_limits}, {wing_joint_1.motion_limits}",
    )
    ctx.check(
        "support links have limited hinge motion",
        link_joint_0.motion_limits.lower < 0.0
        and link_joint_0.motion_limits.upper > 0.0
        and link_joint_1.motion_limits.lower < 0.0
        and link_joint_1.motion_limits.upper > 0.0,
        details=f"limits={link_joint_0.motion_limits}, {link_joint_1.motion_limits}",
    )

    rest_0 = ctx.part_world_aabb(wing_0)
    rest_1 = ctx.part_world_aabb(wing_1)
    with ctx.pose({wing_joint_0: wing_joint_0.motion_limits.upper, wing_joint_1: wing_joint_1.motion_limits.upper}):
        folded_0 = ctx.part_world_aabb(wing_0)
        folded_1 = ctx.part_world_aabb(wing_1)
    ctx.check(
        "wing folding raises free rails",
        rest_0 is not None
        and rest_1 is not None
        and folded_0 is not None
        and folded_1 is not None
        and folded_0[1][2] > rest_0[1][2] + 0.30
        and folded_1[1][2] > rest_1[1][2] + 0.30,
        details=f"rest={rest_0}, {rest_1}; folded={folded_0}, {folded_1}",
    )

    return ctx.report()


object_model = build_object_model()
