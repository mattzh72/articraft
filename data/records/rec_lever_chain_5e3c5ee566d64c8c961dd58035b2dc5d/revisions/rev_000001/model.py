from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="offset_clevis_lever_linkage")

    steel = model.material("steel", rgba=(0.33, 0.36, 0.39, 1.0))
    pin_metal = model.material("pin_metal", rgba=(0.73, 0.75, 0.78, 1.0))
    tab_finish = model.material("tab_finish", rgba=(0.48, 0.51, 0.56, 1.0))

    pin_radius = 0.0042
    hole_radius = pin_radius
    eye_radius = 0.014
    cheek_thickness = 0.008
    link_thickness = 0.007
    cheek_gap = 0.010
    eye_spacing = 0.010
    body_height = 0.014
    head_radius = 0.0068
    head_length = 0.0025

    primary_length = 0.160
    secondary_length = 0.120
    terminal_length = 0.085
    terminal_tab_length = 0.030

    clevis_positive_y = cheek_gap / 2.0 + cheek_thickness / 2.0
    clevis_negative_y = -clevis_positive_y

    def union_all(*items: cq.Workplane) -> cq.Workplane:
        result = items[0]
        for item in items[1:]:
            result = result.union(item)
        return result

    def cylinder_y(radius: float, length: float, *, x: float, y: float, z: float = 0.0) -> cq.Workplane:
        return (
            cq.Workplane("XZ")
            .center(x, z)
            .circle(radius)
            .extrude(length / 2.0, both=True)
            .translate((0.0, y, 0.0))
        )

    def block_xz(
        length_x: float,
        height_z: float,
        span_y: float,
        *,
        x: float,
        y: float,
        z: float = 0.0,
    ) -> cq.Workplane:
        return (
            cq.Workplane("XZ")
            .center(x, z)
            .rect(length_x, height_z)
            .extrude(span_y / 2.0, both=True)
            .translate((0.0, y, 0.0))
        )

    def slot_xz(
        length_x: float,
        height_z: float,
        span_y: float,
        *,
        x: float,
        y: float,
        z: float = 0.0,
    ) -> cq.Workplane:
        return (
            cq.Workplane("XZ")
            .center(x, z)
            .slot2D(length_x, height_z)
            .extrude(span_y / 2.0, both=True)
            .translate((0.0, y, 0.0))
        )

    def eye_lug(*, x: float, y: float, thickness: float) -> cq.Workplane:
        lug = cylinder_y(eye_radius, thickness, x=x, y=y)
        hole = cylinder_y(hole_radius, thickness + 0.010, x=x, y=y)
        return lug.cut(hole)

    def pin_with_head(
        *,
        x: float,
        y_start: float,
        y_end: float,
        head_side: str,
    ) -> cq.Workplane:
        shank = cylinder_y(pin_radius, abs(y_end - y_start), x=x, y=(y_start + y_end) / 2.0)
        if head_side == "positive":
            head_center_y = max(y_start, y_end) + head_length / 2.0
        else:
            head_center_y = min(y_start, y_end) - head_length / 2.0
        head = cylinder_y(head_radius, head_length, x=x, y=head_center_y)
        return shank.union(head)

    def body_bundle(
        *,
        length: float,
        prox_y: float,
        body_y: float,
        dist_y: float,
    ) -> cq.Workplane:
        core_margin = eye_radius * 0.82
        web_length = min(0.028, length * 0.24)
        main_bar = block_xz(
            max(length - 2.0 * core_margin, 0.020),
            body_height,
            link_thickness,
            x=length / 2.0,
            y=body_y,
        )
        prox_web = block_xz(
            web_length,
            body_height,
            link_thickness + abs(body_y - prox_y),
            x=eye_radius * 0.62 + web_length / 2.0,
            y=(body_y + prox_y) / 2.0,
        )
        dist_web = block_xz(
            web_length,
            body_height,
            link_thickness + abs(body_y - dist_y),
            x=length - eye_radius * 0.62 - web_length / 2.0,
            y=(body_y + dist_y) / 2.0,
        )
        return union_all(main_bar, prox_web, dist_web)

    root = model.part("root_clevis")
    root_cheek_length = 0.048
    root_cheek_height = 0.036
    root_bridge = block_xz(
        0.014,
        0.024,
        cheek_gap + 2.0 * cheek_thickness,
        x=-0.030,
        y=0.0,
        z=0.0,
    )
    root_foot = block_xz(
        0.030,
        0.012,
        cheek_gap + 2.0 * cheek_thickness + 0.006,
        x=-0.026,
        y=0.0,
        z=-0.024,
    )
    positive_cheek = slot_xz(
        root_cheek_length,
        root_cheek_height,
        cheek_thickness,
        x=-root_cheek_length / 2.0,
        y=clevis_positive_y,
    ).cut(cylinder_y(hole_radius, cheek_thickness + 0.010, x=0.0, y=clevis_positive_y))
    negative_cheek = slot_xz(
        root_cheek_length,
        root_cheek_height,
        cheek_thickness,
        x=-root_cheek_length / 2.0,
        y=clevis_negative_y,
    ).cut(cylinder_y(hole_radius, cheek_thickness + 0.010, x=0.0, y=clevis_negative_y))
    root_pin = pin_with_head(
        x=0.0,
        y_start=clevis_positive_y + cheek_thickness / 2.0,
        y_end=clevis_negative_y - cheek_thickness / 2.0,
        head_side="positive",
    )
    root.visual(
        mesh_from_cadquery(positive_cheek, "root_clevis_positive_cheek"),
        material=steel,
        name="positive_cheek",
    )
    root.visual(
        mesh_from_cadquery(negative_cheek, "root_clevis_negative_cheek"),
        material=steel,
        name="negative_cheek",
    )
    root.visual(
        mesh_from_cadquery(union_all(root_bridge, root_foot), "root_clevis_root_body"),
        material=steel,
        name="root_body",
    )
    root.visual(
        mesh_from_cadquery(root_pin, "root_clevis_root_pin"),
        material=pin_metal,
        name="root_pin",
    )

    primary = model.part(
        "primary_lever",
        meta={"nominal_length": primary_length},
    )
    primary_prox_y = 0.0
    primary_body_y = -0.014
    primary_dist_y = -eye_spacing
    primary_prox_eye = eye_lug(x=0.0, y=primary_prox_y, thickness=link_thickness)
    primary_dist_eye = eye_lug(x=primary_length, y=primary_dist_y, thickness=link_thickness)
    primary_body = body_bundle(
        length=primary_length,
        prox_y=primary_prox_y,
        body_y=primary_body_y,
        dist_y=primary_dist_y,
    )
    primary_pin = pin_with_head(
        x=primary_length,
        y_start=primary_dist_y - link_thickness / 2.0,
        y_end=eye_spacing + link_thickness / 2.0,
        head_side="negative",
    )
    primary.visual(
        mesh_from_cadquery(primary_prox_eye, "primary_lever_prox_eye"),
        material=steel,
        name="prox_eye",
    )
    primary.visual(
        mesh_from_cadquery(primary_body, "primary_lever_body"),
        material=steel,
        name="body",
    )
    primary.visual(
        mesh_from_cadquery(primary_dist_eye, "primary_lever_dist_eye"),
        material=steel,
        name="dist_eye",
    )
    primary.visual(
        mesh_from_cadquery(primary_pin, "primary_lever_joint_pin"),
        material=pin_metal,
        name="joint_pin",
    )

    secondary = model.part(
        "secondary_lever",
        meta={"nominal_length": secondary_length},
    )
    secondary_prox_y = eye_spacing
    secondary_body_y = 0.0
    secondary_dist_y = -eye_spacing
    secondary_prox_eye = eye_lug(x=0.0, y=secondary_prox_y, thickness=link_thickness)
    secondary_dist_eye = eye_lug(x=secondary_length, y=secondary_dist_y, thickness=link_thickness)
    secondary_body = body_bundle(
        length=secondary_length,
        prox_y=secondary_prox_y,
        body_y=secondary_body_y,
        dist_y=secondary_dist_y,
    )
    secondary_pin = pin_with_head(
        x=secondary_length,
        y_start=secondary_dist_y - link_thickness / 2.0,
        y_end=eye_spacing + link_thickness / 2.0,
        head_side="negative",
    )
    secondary.visual(
        mesh_from_cadquery(secondary_prox_eye, "secondary_lever_prox_eye"),
        material=steel,
        name="prox_eye",
    )
    secondary.visual(
        mesh_from_cadquery(secondary_body, "secondary_lever_body"),
        material=steel,
        name="body",
    )
    secondary.visual(
        mesh_from_cadquery(secondary_dist_eye, "secondary_lever_dist_eye"),
        material=steel,
        name="dist_eye",
    )
    secondary.visual(
        mesh_from_cadquery(secondary_pin, "secondary_lever_joint_pin"),
        material=pin_metal,
        name="joint_pin",
    )

    terminal = model.part(
        "terminal_lever",
        meta={"nominal_length": terminal_length},
    )
    terminal_prox_y = eye_spacing
    terminal_body_y = 0.014
    terminal_tab_y = 0.016
    terminal_prox_eye = eye_lug(x=0.0, y=terminal_prox_y, thickness=link_thickness)
    terminal_body = block_xz(
        terminal_length - eye_radius * 0.95,
        body_height * 0.92,
        link_thickness,
        x=(terminal_length - eye_radius * 0.95) / 2.0 + eye_radius * 0.55,
        y=terminal_body_y,
    ).union(
        block_xz(
            min(0.028, terminal_length * 0.32),
            body_height * 0.92,
            link_thickness + abs(terminal_body_y - terminal_prox_y),
            x=eye_radius * 0.62 + min(0.028, terminal_length * 0.32) / 2.0,
            y=(terminal_body_y + terminal_prox_y) / 2.0,
        )
    )
    terminal_tab = slot_xz(
        terminal_tab_length,
        0.012,
        link_thickness,
        x=terminal_length + terminal_tab_length / 2.0 - 0.004,
        y=terminal_tab_y,
    ).cut(
        cylinder_y(
            0.0036,
            link_thickness + 0.010,
            x=terminal_length + terminal_tab_length * 0.60,
            y=terminal_tab_y,
        )
    )
    terminal_tab_web = block_xz(
        0.020,
        0.012,
        link_thickness + abs(terminal_tab_y - terminal_body_y),
        x=terminal_length - 0.004,
        y=(terminal_tab_y + terminal_body_y) / 2.0,
    )
    terminal.visual(
        mesh_from_cadquery(terminal_prox_eye, "terminal_lever_prox_eye"),
        material=steel,
        name="prox_eye",
    )
    terminal.visual(
        mesh_from_cadquery(terminal_body.union(terminal_tab_web), "terminal_lever_body"),
        material=steel,
        name="body",
    )
    terminal.visual(
        mesh_from_cadquery(terminal_tab, "terminal_lever_terminal_tab"),
        material=tab_finish,
        name="terminal_tab",
    )

    model.articulation(
        "root_to_primary",
        ArticulationType.REVOLUTE,
        parent=root,
        child=primary,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=2.5, lower=-1.15, upper=1.15),
    )
    model.articulation(
        "primary_to_secondary",
        ArticulationType.REVOLUTE,
        parent=primary,
        child=secondary,
        origin=Origin(xyz=(primary_length, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.5, lower=-1.35, upper=1.35),
    )
    model.articulation(
        "secondary_to_terminal",
        ArticulationType.REVOLUTE,
        parent=secondary,
        child=terminal,
        origin=Origin(xyz=(secondary_length, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.5, lower=-1.20, upper=1.20),
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

    root = object_model.get_part("root_clevis")
    primary = object_model.get_part("primary_lever")
    secondary = object_model.get_part("secondary_lever")
    terminal = object_model.get_part("terminal_lever")

    root_to_primary = object_model.get_articulation("root_to_primary")
    primary_to_secondary = object_model.get_articulation("primary_to_secondary")
    secondary_to_terminal = object_model.get_articulation("secondary_to_terminal")

    for part in (root, primary, secondary, terminal):
        ctx.check(f"{part.name} exists", part is not None)

    for joint in (root_to_primary, primary_to_secondary, secondary_to_terminal):
        ctx.check(
            f"{joint.name} uses a shared pin axis",
            tuple(joint.axis) == (0.0, -1.0, 0.0),
            details=f"axis={joint.axis}",
        )

    ctx.check(
        "lever lengths step down along the chain",
        (
            primary.meta.get("nominal_length", 0.0)
            > secondary.meta.get("nominal_length", 0.0)
            > terminal.meta.get("nominal_length", 0.0)
        ),
        details=(
            f"primary={primary.meta.get('nominal_length')}, "
            f"secondary={secondary.meta.get('nominal_length')}, "
            f"terminal={terminal.meta.get('nominal_length')}"
        ),
    )

    ctx.allow_overlap(
        root,
        primary,
        elem_a="root_pin",
        elem_b="prox_eye",
        reason=(
            "The root clevis pin is intentionally modeled as occupying the "
            "primary lever bore to represent the first revolute pin joint."
        ),
    )
    ctx.allow_overlap(
        primary,
        secondary,
        elem_a="joint_pin",
        elem_b="prox_eye",
        reason=(
            "The primary lever pin is intentionally modeled inside the "
            "secondary lever eye to represent the second revolute joint."
        ),
    )
    ctx.allow_overlap(
        secondary,
        terminal,
        elem_a="joint_pin",
        elem_b="prox_eye",
        reason=(
            "The secondary lever pin is intentionally modeled inside the "
            "terminal lever eye to represent the third revolute joint."
        ),
    )

    with ctx.pose({root_to_primary: 0.0, primary_to_secondary: 0.0, secondary_to_terminal: 0.0}):
        ctx.expect_overlap(
            root,
            primary,
            axes="xz",
            elem_a="root_pin",
            elem_b="prox_eye",
            min_overlap=0.008,
            name="root pin aligns with the primary lever eye",
        )
        ctx.expect_gap(
            root,
            primary,
            axis="y",
            positive_elem="positive_cheek",
            negative_elem="prox_eye",
            min_gap=0.001,
            max_gap=0.003,
            name="positive clevis cheek clears the primary lever eye",
        )
        ctx.expect_gap(
            primary,
            root,
            axis="y",
            positive_elem="prox_eye",
            negative_elem="negative_cheek",
            min_gap=0.001,
            max_gap=0.003,
            name="negative clevis cheek clears the primary lever eye",
        )

        ctx.expect_overlap(
            primary,
            secondary,
            axes="xz",
            elem_a="joint_pin",
            elem_b="prox_eye",
            min_overlap=0.008,
            name="primary link pin aligns with the secondary lever eye",
        )
        ctx.expect_overlap(
            primary,
            secondary,
            axes="xz",
            elem_a="dist_eye",
            elem_b="prox_eye",
            min_overlap=0.020,
            name="primary and secondary eyes share the second joint centerline",
        )
        ctx.expect_gap(
            secondary,
            primary,
            axis="y",
            positive_elem="prox_eye",
            negative_elem="dist_eye",
            min_gap=0.012,
            max_gap=0.015,
            name="secondary lever sits outboard from the primary lever at joint two",
        )

        ctx.expect_overlap(
            secondary,
            terminal,
            axes="xz",
            elem_a="joint_pin",
            elem_b="prox_eye",
            min_overlap=0.008,
            name="secondary link pin aligns with the terminal lever eye",
        )
        ctx.expect_overlap(
            secondary,
            terminal,
            axes="xz",
            elem_a="dist_eye",
            elem_b="prox_eye",
            min_overlap=0.020,
            name="secondary and terminal eyes share the third joint centerline",
        )
        ctx.expect_gap(
            terminal,
            secondary,
            axis="y",
            positive_elem="prox_eye",
            negative_elem="dist_eye",
            min_gap=0.012,
            max_gap=0.015,
            name="terminal lever sits outboard from the secondary lever at joint three",
        )

    def aabb_center(aabb):
        if aabb is None:
            return None
        lower, upper = aabb
        return tuple((a + b) / 2.0 for a, b in zip(lower, upper))

    rest_tab_center = aabb_center(ctx.part_element_world_aabb(terminal, elem="terminal_tab"))
    with ctx.pose({root_to_primary: 0.55, primary_to_secondary: 0.35, secondary_to_terminal: 0.30}):
        posed_tab_center = aabb_center(ctx.part_element_world_aabb(terminal, elem="terminal_tab"))

    ctx.check(
        "terminal tab lifts upward when the linkage flexes",
        (
            rest_tab_center is not None
            and posed_tab_center is not None
            and posed_tab_center[2] > rest_tab_center[2] + 0.05
        ),
        details=f"rest={rest_tab_center}, posed={posed_tab_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
